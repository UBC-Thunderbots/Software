#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_candidate.h"

template<typename TSupportTactic>
SupportTacticCandidate<TSupportTactic>::SupportTacticCandidate() 
    : scores_(), total_score_(0.0), total_score_invalidated_(false)
{
}

template<typename TSupportTactic>
void SupportTacticCandidate<TSupportTactic>::score(SupportTacticScorer &scorer)
{
    // We don't care about scores outside the range [-1.0, 1.0]
    double score = std::clamp(scorer.score(*this), -1.0, 1.0);
    scores_.push_back(score);

    // Indicate that total score needs to be recomputed
    total_score_invalidated_ = true;
}

template<typename TSupportTactic>
void SupportTacticCandidate<TSupportTactic>::resetTotalScore()
{
    scores_.clear();
    total_score_ = 0.0;
}

template<typename TSupportTactic>
double SupportTacticCandidate<TSupportTactic>::getTotalScore()
{
    if (total_score_invalidated_) 
    {
        computeTotalScore();
        total_score_invalidated_ = false;
    }
    
    return total_score_;
}

template<typename TSupportTactic>
std::shared_ptr<TSupportTactic> SupportTacticCandidate<TSupportTactic>::create() const
{
    return std::make_shared<TSupportTactic>();
}

template<typename TSupportTactic>
void SupportTacticCandidate<TSupportTactic>::computeTotalScore()
{
    // Total score is calculated as the generalized mean of the 
    // scores applied to the candidate

    // See javadoc comment for computeTotalScore
    
    double sum = std::accumulate(scores_.begin(), scores_.end(), 0.0,
        [](double current_sum, double score) {
            double sign = (0.0 < score) - (score < 0.0); 
            double term = sign * std::pow(std::abs(score), 1.0 / SINGLE_SCORE_INFLUENCE);
            return current_sum + term;
        });

    double num_scores = std::static_cast<double>(scores_.size());
    total_score_ = std::pow(sum / num_scores, SINGLE_SCORE_INFLUENCE);
}
