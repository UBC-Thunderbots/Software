#pragma once

#include <vector>

/**
 * A Candidate represents something that can be scored. 
 *
 * Individual scores can be applied to a Candidate, and the Candidate
 * will return a total score summarizing all the individual scores
 * applied to it.
 */
class Candidate
{
   public:
    /**
     * Gets the total score for this candidate, which summarizes all
     * of the individual scores applied to this candidate
     * 
     * @return the total score for this candidate
     */
    double getTotalScore();

    /**
     * Removes all scores applied to this candidate
     */
    void clearScores();

   protected:
    explicit Candidate();

    /**
     * Applies a score to this candidate
     *
     * @param score the score to apply to this candidate
     */
    void applyScore(double score);

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
};
