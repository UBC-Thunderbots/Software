#pragma once

#include <set>

#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_scorer.h"
#include "software/ai/hl/stp/tactic/pass_receiver/pass_receiver_tactic.h"
#include "software/ai/hl/stp/tactic/fake_pass_receiver/fake_pass_receiver_tactic.h"
#include "software/ai/hl/stp/tactic/cherry_picker/cherry_picker_tactic.h"
#include "software/ai/hl/stp/tactic/disrupter/disrupter_tactic.h"

/**
 * A SupportTacticCandidate is a "type wrapper" that makes a type of tactic
 * eligible for scoring and assignment as a support tactic in a DynamicPlay.
 *
 * It acts as the visitee for a SupportTacticScorer, which operates under
 * the visitor pattern.
 */
template<typename TSupportTactic>
class SupportTacticCandidate 
{
    static_assert(std::is_base_of<Tactic, TSupportTactic>::value, 
                  "TSupportTactic must derive from Tactic");

   public:
    explicit SupportTacticCandidate();

    /**
     * Accepts a SupportTacticScorer and calls the scorer's `score` function 
     * on itself. The scorer's returned score is applied to this candidate.
     *
     * @param scorer a SupportTacticScorer to use to score this candidate
     */
    void score(SupportTacticScorer &scorer);

    /**
     * Removes all scores applied to this candidate
     */
    void clearScores();

    /**
     * Gets the total score for this candidate, which summarizes all
     * of the individual scores applied to this candidate
     * 
     * @return the total score for this candidate
     */
    double getTotalScore();

    /**
     * Returns a shared pointer to a newly constructed instance of the
     * candidate's support tactic type
     *
     * @return a shared pointer to a newly constructed instance of the
     * candidate's support tactic type
     */
    std::shared_ptr<TSupportTactic> createSupportTactic() const;

   private:
    // The individual scores applied to this candidate
    std::vector<double> scores_;

    // The total score for this candidate, cached to reduce repeated
    // calculation upon calls to getTotalScore
    double total_score_;

    // Flag to indicate whether total score needs to be recomputed
    bool total_score_invalidated_;

    /**
     * Computes and updates the total score for this candidate.
     * 
     * The total score X(x_1, x_2, ..., x_n) summarizes all the individual 
     * scores x_i that have been applied to this candidate. It is calculated 
     * as a generalized mean of the form 
     *
     *                                  n
     * X(x_1, x_2, ..., x_n) = ((1/n) * Σ sign(x_i) * |x_i|^(1/p))^p
     *                                 i=1
     *
     * where the parameter p is > 0. As p gets larger, scores close to 0
     * and negative scores will influence the total score more than values 
     * far away from 0. 
     *
     * This parameter p is useful to us since negative scores are meant
     * to penalize and scores close to 0 indicate low viability, so they
     * should disproportionately impact the total score more so than any 
     * single large positive score.
     */
    void computeTotalScore();

    // Parameter p in total score calculation
    static constexpr double SINGLE_SCORE_INFLUENCE = 2.5;
}

using SupportTacticCandidateVector = std::vector<std::shared_ptr<SupportTacticCandidate<Tactic>>>;

/**
 * Returns all support tactic candidates
 * 
 * @return a vector of shared pointers to all support tactic candidates
 */
static SupportTacticCandidateVector allSupportTacticCandidates() 
{
    return 
    {
        std::make_shared<SupportTacticCandidate<PassReceiverTactic>>(),
        std::make_shared<SupportTacticCandidate<FakePassReceiverTactic>>(),
        std::make_shared<SupportTacticCandidate<CherryPickerTactic>>(),
        std::make_shared<SupportTacticCandidate<DisrupterTactic>>()
    };
}

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
