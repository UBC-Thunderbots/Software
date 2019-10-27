#include "software/particle_filter.h"

#include "software/geom/util.h"
#include "software/util/parameter/dynamic_parameters.h"

/* Notes on the weights and evaluation:
 * - PREDICTION_WEIGHT should always be greater than PREVIOUS_BALL_WEIGHT
 * because
 *   in the case where there is no vision detection, we want to take the
 * prediction position not
 *   the old ball position so the ball keeps moving (we don't want the ball to
 * stop if we lose vision for a second).
 *
 * - If there are vision detections, one of them should be chosen (this should
 * be 99% of all the cases, unless
 *   they are all really far away and therefore likely to be just noise) and the
 * ball position and prediction
 *   weights should just be used to help weight the particles closer to the
 * detection closest to the ball
 *   (which should be the detection for the real ball)
 */

ParticleFilter::ParticleFilter(double length, double width)
{
    length_ = length;
    width_  = width;

    // initialize vector with PARTICLE_FILTER_NUM_PARTICLES Particle() objects
    particles = std::vector<Particle>(num_particles, Particle());

    // Set the seed for the random number generator
    seed = static_cast<unsigned int>(time(NULL));

    // These will be used to generate Points with a gaussian distribution
    generator = std::default_random_engine(seed);

    // This will be used to generate Points spread across the whole field
    linearGenerator = std::minstd_rand0(seed);

    // These start with the placeholder so we can tell we haven't detected a
    // valid ball yet
    ballPosition          = TMP_POINT;
    ballPredictedPosition = TMP_POINT;
    ballPositionVariance  = 0.0;

    ballConfidence = 0.0;
}

void ParticleFilter::add(const Point pos)
{
    // We only care about points that are within the field. The ball can't
    // be outside the field, and we don't need to track it there
    if (!std::isnan(pos.x() + pos.y()) && isInField(pos))
    {
        detections.push_back(pos);
    }
}

void ParticleFilter::update(Point ballPredictedPos)
{
    ballPredictedPosition = ballPredictedPos;
    basepoints            = detections;

    // Only add the PredictedPosition (not the ball's last position) as a
    // basepoint because the ball is more likely
    // to be there than the ball's old location. If the ball is not moving, the
    // prediction should be the ball's position anyway.
    if (ballPosition != TMP_POINT && ballPredictedPosition != TMP_POINT &&
        isInField(ballPredictedPosition))
    {
        basepoints.push_back(ballPredictedPos);
    }

    // Because we only add points that are inside the field to the list of
    // basepoints, at this point
    // we can guarantee that all basepoints are within the field, so don't need
    // to do another loop
    // to check them

    // This is the main particle filter loop.
    // - Generate particles around the given basepoints
    // - Update the confidences of these new particles
    // - Evaluate each particle, update their confidence, and keep the top
    // percentage of particles to use
    // as basepoints for the next iteration. These particles should converge to
    // the ball's location.
    int num_condensations = Util::DynamicParameters->getNetworkInput()
                                ->getFilterConfig()
                                ->getParticleFilterConfig()
                                ->NumCondensations()
                                ->value();

    double top_percentage_of_particles = Util::DynamicParameters->getNetworkInputConfig()
                                             ->getFilterConfig()
                                             ->getParticleFilterConfig()
                                             ->TopPercentageOfParticles()
                                             ->value();

    for (int i = 0; i < num_condensations; i++)
    {
        // As we loop through each condensation, we want the particles that are
        // generated around the basepoints
        // to get more and more precise, so that we can pinpoint a more accurate
        // point when we take the mean

        // Make sure the denominator is never negative or 0
        double particle_standard_dev_decrement_denominator =
            num_condensations < 1 ? 1 : num_condensations - 1;
        double particle_standard_dev_decrement =
            (max_particle_standard_dev - min_particle_standard_dev) /
            particle_standard_dev_decrement_denominator;
        double particle_standard_dev =
            max_particle_standard_dev - i * particle_standard_dev_decrement;

        generateParticles(basepoints, particle_standard_dev);
        updateParticleConfidences();

        unsigned int numParticlesToKeep =
            static_cast<unsigned int>(ceil(top_percentage_of_particles * num_particles));

        // make sure we never try keep more particles than we have
        if (numParticlesToKeep > particles.size())
        {
            numParticlesToKeep = static_cast<unsigned int>(particles.size());
        }

        // sort the list of Particles by their confidences from least to most
        // confidence
        std::sort(particles.begin(), particles.end());

        // Replace the basepoints with the positions of the most confident
        // particles. We iterate over the last part of the array since this is
        // where the particles
        // with the most confidence are because of the sort
        basepoints.clear();
        for (auto it = particles.end() - numParticlesToKeep; it != particles.end(); it++)
        {
            basepoints.push_back(it->position);
        }
    }

    // Average the final basepoints to get the ball's location. This makes the
    // ball's
    // movement slightly smoother than just taking the single most confident
    // point
    Point newBallPosition          = getPointsMean(basepoints);
    double newBallPositionVariance = getPointsVariance(basepoints);

    // If there are no balls detected, the ball could be covered, be being
    // moved,
    // or be off the field, and we don't know. If we already don't have
    // confidence in
    // the ball's location, we use the old position since we can't find a new
    // ball to
    // be more confident in. Otherwise, we lose confidence in the current ball
    // but still use
    // the predicted position, so we are tolerant to the ball disappearing for a
    // few frames.
    double ball_confidence_threshold = Util::DynamicParameters->getNetworkInputConfig()
                                           ->getFilterConfig()
                                           ->getParticleFilterConfig()
                                           ->BallConfidenceThreshold.value();
    double ball_confidence_delta = Util::DynamicParameters->getNetworkInputConfig()
                                       ->getFilterConfig()
                                       ->getParticleFilterConfig->BallConfidenceDelta()
                                       ->value();
    double ball_valid_dist_threshold = Util::DynamicParameters->getNetworkInputConfig()
                                           ->getFilterConfig()
                                           ->getParticleFilterConfig()
                                           ->BallValidDistThreshold()
                                           ->value();
    double ball_max_variance = Util::DynamicParameters->getNetworkInputConfig()
                                   ->getFilterConfig()
                                   ->getParticleFilterConfig()
                                   ->BallMaxVariance()
                                   ->value();
    if (detections.empty())
    {
        if (ballConfidence < ball_confidence_threshold)
        {
            updateBallConfidence(-ball_confidence_delta);
            ballPosition         = ballPosition;
            ballPositionVariance = ballPositionVariance;
        }
        else
        {
            updateBallConfidence(-ball_confidence_delta);
            ballPosition         = newBallPosition;
            ballPositionVariance = newBallPositionVariance;
        }
    }
    // If the ball suddenly moved a large distance, or the variance of the
    // newest ball detection
    // is very high, something might be wrong with our latest ball
    // detection/filter. In this case we
    // lose some confidence in our current ball detection and use the ball's
    // predicted position rather
    // than the filtered on since it's more likely to be correct. If we already
    // don't have enough confidence
    // in the ball, we use the ball's filtered position since if we don't, we
    // can get stuck in this "state"
    // if the ball reappears far away.

    else if ((newBallPosition - ballPosition).len() > ball_valid_dist_threshold ||
             newBallPositionVariance > ball_max_variance)
    {
        if (ballConfidence < ball_confidence_threshold)
        {
            updateBallConfidence(-ball_confidence_delta);
            ballPosition         = newBallPosition;
            ballPositionVariance = newBallPositionVariance;
        }
        else
        {
            updateBallConfidence(-ball_confidence_delta);
            ballPosition         = ballPredictedPosition;
            ballPositionVariance = newBallPositionVariance;
        }
    }
    // In this case, we have good data to filter with. We use the ball's
    // predicted position
    // and increase our confidence in the ball.
    else
    {
        ballPosition         = newBallPosition;
        ballPositionVariance = newBallPositionVariance;
        updateBallConfidence(ball_confidence_delta);
    }

    detections.clear();  // Clear the detections for the next tick
}

void ParticleFilter::generateParticles(const std::vector<Point> &basepoints,
                                       double standard_dev)
{
    if (basepoints.empty())
    {
        // If there are no basepoints, spread random points across the whole
        // field
        for (unsigned int i = 0; i < particles.size(); i++)
        {
            double x = static_cast<double>(linearGenerator()) /
                           ((double)linearGenerator.max() / length_) -
                       length_ / 2;
            double y = static_cast<double>(linearGenerator()) /
                           ((double)linearGenerator.max() / width_) -
                       width_ / 2;
            particles[i].position = Point(x, y);
        }
    }
    else
    {
        std::normal_distribution<double> particleNormalDistribution =
            std::normal_distribution<double>(0.0, standard_dev);

        // If there are basepoints, generate points around them with a gaussian
        // distribution
        int count;
        for (unsigned int i = 0; i < particles.size(); i++)
        {
            Point basepoint   = basepoints[static_cast<unsigned int>(
                i / (particles.size() / basepoints.size()))];
            Point newParticle = Point();
            count             = 0;
            do
            {
                double x = particleNormalDistribution(generator) + basepoint.x();
                double y = particleNormalDistribution(generator) + basepoint.y();

                newParticle = Point(x, y);
                count++;
            } while (!isInField(newParticle) && count < 10);

            // If the generated particle is not within the field, try to
            // generate another one.
            // We don't care about points outside the field. If we are unable to
            // produce a particle inside
            // the field within 10 attempts, just generate the particle
            // somewhere on the field. 10 is chosen
            // arbitrarily here, so that we get several attempts but don't slow
            // down the algorithm.

            if (count >= 10)
            {
                // TODO: remove this duplicate code
                // failed- just generate a uniformly distributed new particle
                double x = static_cast<double>(linearGenerator()) /
                               ((double)linearGenerator.max() / length_) -
                           length_ / 2;
                double y = static_cast<double>(linearGenerator()) /
                               ((double)linearGenerator.max() / width_) -
                           width_ / 2;
                newParticle = Point(x, y);
            }

            particles[i].position = newParticle;
        }
    }
}

void ParticleFilter::updateBallConfidence(double val)
{
    double newConfidence = ballConfidence + val;
    if (newConfidence > max_ball_confidence)
    {
        ballConfidence = max_ball_confidence;
    }
    else if (newConfidence < 0.0)
    {
        ballConfidence = 0.0;
    }
    else
    {
        ballConfidence = newConfidence;
    }
}

void ParticleFilter::updateParticleConfidences()
{
    for (unsigned int k = 0; k < particles.size(); k++)
    {
        particles[k].confidence = evaluateParticle(particles[k].position);
    }
}

double ParticleFilter::evaluateParticle(const Point &particle)
{
    double detectionScore       = 0.0;
    double max_detection_weight = Util::DynamicParameters->getNetworkInputConfig()
                                      ->getFilterConfig()
                                      ->getParticleFilterConfig()
                                      ->MaxDetectionWeight()
                                      ->value();
    double previous_ball_weight = Util::DynamicParameters->getNetworkInputConfig()
                                      ->getFilterConfig()
                                      ->getParticleFilterConfig()
                                      ->PreviousBallWeight()
                                      ->value();
    double ball_dist_threshold = Util::DynamicParameters->getNetworkInputConfig()
                                     ->getFilterConfig()
                                     ->getParticleFilterConfig()
                                     ->BallDistThreshold()
                                     ->value();
    double prediction_weight = Util::DynamicParameters->getNetworkInputConfig()
                                   ->getFilterConfig()
                                   ->getParticleFilterConfig()
                                   ->PredictionWeight()
                                   ->value();

    for (unsigned int i = 0; i < detections.size(); i++)
    {
        double detectionDist = (particle - detections[i]).len();
        if (ballPosition != TMP_POINT)
        {
            detectionScore += getDetectionWeight((detections[i] - ballPosition).len()) *
                              exp(-detectionDist);
        }
        else
        {
            detectionScore += max_detection_weight * exp(-detectionDist);
        }
    }

    double previousBallScore = 0.0;


    if (ballPosition != TMP_POINT)
    {
        double ballDist = (particle - ballPosition).len();
        // This is an older equation from development. It should work also, but
        // the last working test that was
        // done was with the sqrt function so we're using it for now.
        //		previousBallScore += PREVIOUS_BALL_WEIGHT * exp(-0.5 *
        // ballDist);

        // This weight will drop to 0 if ballDist is greater than
        // BALL_DIST_THRESHOLD
        previousBallScore += previous_ball_weight * sqrt(-ballDist + ball_dist_threshold);
    }

    double predictionScore = 0.0;
    if (ballPosition != TMP_POINT && ballPredictedPosition != TMP_POINT)
    {
        double predictionDist = (particle - ballPredictedPosition).len();
        // This is an older equation from development. It should work also, but
        // the last working test that was
        // done was with the sqrt function so we're using it for now.
        //		predictionScore += PREDICTION_WEIGHT *
        // exp(-predictionDist);

        // This weight will drop to 0 if ballDist is greater than
        // BALL_DIST_THRESHOLD. 3 is a (somewhat arbitrary) constant to make
        // sure
        // the weight doesn't drop to 0 unless the ball is even further away.
        // Since the ball could bounce in the opposite direction of the
        // prediction, we still want reasonable bounces to gain weight from this
        // function.
        predictionScore +=
            prediction_weight * sqrt(-predictionDist + ball_dist_threshold * 3);
    }

    return detectionScore + previousBallScore + predictionScore;
}

double ParticleFilter::getDetectionWeight(const double dist)
{
    double detection_weight_decay = Util::DynamicParameters->getNetworkInputConfig()
                                        ->getFilterConfig()
                                        ->getParticleFilterConfig()
                                        ->DetectionWeightDecay()
                                        ->value();
    double max_detection_weight = Util::DynamicParameters->getNetworkInputConfig()
                                      ->getFilterConfig()
                                      ->getParticleFilterConfig()
                                      ->MaxDetectionWeight()
                                      ->value();
    double weight = max_detection_weight - detection_weight_decay * dist;
    return weight < 0.0 ? 0.0 : weight;
}

bool ParticleFilter::isInField(const Point &p)
{
    return fabs(p.x()) <= length_ / 2 && fabs(p.y()) <= width_ / 2;
}

Point ParticleFilter::getEstimate()
{
    return ballPosition == TMP_POINT ? Point() : ballPosition;
}

double ParticleFilter::getEstimateVariance()
{
    return ballPositionVariance;
}
