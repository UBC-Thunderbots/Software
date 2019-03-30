#ifndef NETWORK_INPUT_FILTER_PARTICLE_FILTER_H
#define NETWORK_INPUT_FILTER_PARTICLE_FILTER_H

#include <random>

#include "geom/point.h"
#include "util/parameter/dynamic_parameters.h"

/**
 * Finds and filters the "real" ball from the received data
 */
const unsigned int num_particles       = 500;
const double max_ball_confidence       = 100.0;
const double max_particle_standard_dev = 0.05;
const double min_particle_standard_dev = 0.001;

// This is used as a placeholder point for when we don't have real data
const Point TMP_POINT = Point(-99.99, -99.99);

/**
 * Defines a particle used by the particle filter.
 *
 * A particle is a point with an associated confidence value that indicates how
 * likely
 * the particle is to be object you are tracking
 */
struct Particle
{
    Particle(Point p = Point(), double c = 0.0) : position(p), confidence(c) {}

    Point position;
    double confidence;

    /**
     * Overrides the less-than operator so that it compares the confidence of
     * the particles
     *
     * Used with std::sort to sort a vector of Particles by their confidence
     * values
     */
    inline bool operator<(const Particle& p) const
    {
        return confidence < p.confidence;
    }
};

/**
 * Implements the basic mathematics of a Particle filter.
 */
class ParticleFilter final
{
   public:
    /**
     * The constructor for the particle filter.
     *
     * @param length the length of the field the particle filter is operating on
     * @param width the width of the field the particle filter is operating on
     */
    explicit ParticleFilter(double length, double width);

    /**
     * Adds a point to the Particle Filter
     *
     * Adds a particle with the given position to the Particle Filter.
     * The particle filter uses these points that are added
     * as the basepoints for generating more particles
     *
     * @param pos the position of the particle to be added
     */
    void add(Point pos);

    /**
     * Updates the state of the Particle Filter, generating new particles and
     * re-evaluating them.
     *
     * Steps:
     * - Generate new particles around the given basepoints (points added with
     * the add() function)
     * - Evaluate each new particle and update its confidence value
     * - Select new basepoints from the particles with the highest confidence
     * - Repeat the above steps for the number of condensations. This should
     *   cause the particles and basepoints to converge to the most confident
     * position (the real ball).
     * - Finally, take the mean of the final, most confident basepoints and use
     * that as the ball location
     *
     * @param ballPredictedPos an optional parameter for the ball's predicted
     * position. The Particle Filter uses this Point to help evaluate the
     * particles,
     * since particles that are closer to the ball's predicted position are more
     * likely
     * to be the ball
     */
    void update(Point ballPredictedPos = TMP_POINT);

    /**
     * Returns the ball's estimated position
     *
     * @return the ball's estimated position
     */
    Point getEstimate();

    /**
     * Returns the variance corresponding to the ball's estimated position
     *
     * @return the variance corresponding to the ball's estimated position
     */
    double getEstimateVariance();

   private:
    // Holds the list of particles the filter uses
    std::vector<Particle> particles;

    // The seed for the random number generators
    unsigned int seed;

    // The generator used with the normal_distrubution to generate
    // values with a gaussian distribution
    std::default_random_engine generator;

    // The generator used to generate random linear values. This is used to
    // generate Particles spread across the whole field
    std::minstd_rand0 linearGenerator;

    // Holds the list of points that are added with the add() function. We can
    // expect these
    // to be any ball positions detected by vision. Essentially, these are all
    // the places the
    // ball "could" be
    std::vector<Point> detections;

    // The points we are the most confident in, and the ones we will use to
    // generate the
    // next set of particles
    std::vector<Point> basepoints;

    // These maintain some state for the ball
    Point ballPosition;
    Point ballPredictedPosition;
    double ballPositionVariance;

    // How confident the filter is that the position it's reporting is the
    // actual position of the ball.
    // This value decays when we have no camera data (since we are only guessing
    // based of off physics where the ball is),
    // and increases when we do have camera data that comes close to our
    // predictions
    double ballConfidence;

    // The filter stores the field size
    double length_;
    double width_;

    /**
     * Generates new particles around the given basepoints
     *
     * Generates PARTICLE_FILTER_NUM_PARTICLES Particles in gaussian
     * distributions around the given basepoints. If no basepoints are given,
     * generates the Particles randomly across the whole field. The particles
     * will be generated within the bounds of the field.
     *
     * @param variance The variance to use for the Gaussian Distribution that
     * the filter uses to generate the particles
     */
    void generateParticles(const std::vector<Point>& basepoints, double standard_dev);

    /**
     * Updates the confidence of each Particle in the list of particles
     *
     * Evaluates each particle in the filter's list of particles and assigns a
     * new confidence value for each.
     */
    void updateParticleConfidences();

    /**
     * Increments or decrements the ball's confidence value by val, keeping the
     * value clamped
     * between 0 and MAX_BALL_CONFIDENCE
     *
     * @param val the amount to update the confidence by
     */
    void updateBallConfidence(double val);

    /**
     * Evaluates the given Point and returns a score based on how likely is it
     * that the ball is at that location
     *
     * Evaluation factors:
     * - Distance from a vision detection (closer is better)
     * - Distance from the ball's previous position (closer is better)
     * - Distance from the ball's previous predicted location (closer is better)
     *
     * @param pos the position of the particle to be evaluated
     */
    double evaluateParticle(const Point& pos);

    /**
     * Returns the Detection Weight as a function of distance from the ball's
     * previous location.
     *
     * @param dist the distance from the ball's last known location
     */
    double getDetectionWeight(const double dist);

    /**
     * Return true if the Point is within the field
     *
     * @param p the point to check
     * @return true if p is within the field and false
     * otherwise
     */
    bool isInField(const Point& p);
};


#endif  // NETWORK_INPUT_FILTER_PARTICLE_FILTER_H
