#pragma once

// What a solution is judged by. Every accept, admission and export comparison goes through the tests below.
namespace score_bpp9000
{

struct Rating
{
    // Timed-out rollout or non-canonical nonce.
    static constexpr unsigned int INVALID_ERROR = 0xFFFFFFFFU;

    unsigned int error;   // wrongs inside the graded frame, lower is better
    unsigned int shift;   // rolling-frame position reached, higher is better

    // False for a timed-out or refused walk.
    bool isValid() const
    {
        return error != INVALID_ERROR;
    }

    // Higher shift wins; equal shift is settled by fewer errors.
    bool isBetterThan(const Rating& other) const
    {
        if (shift != other.shift)
        {
            return shift > other.shift;
        }
        return error < other.error;
    }

    // Frame-0 floor; a node that mastered any frame passes on shift alone.
    bool clearsFloor(unsigned int threshold) const
    {
        return (shift > 0) || (error <= threshold);
    }

    // The anti-attractor's explore test: error only.
    bool errorWorseOrEqual(const Rating& other) const
    {
        return error >= other.error;
    }

    // The exploit test: at least as good, so a flat stretch can be walked across.
    bool isNotWorseThan(const Rating& other) const
    {
        return !other.isBetterThan(*this);
    }

    // Frame 0 with nothing achieved; any real solution beats it.
    static Rating worst()
    {
        Rating r;
        r.error = INVALID_ERROR;
        r.shift = 0;
        return r;
    }
};

static_assert(sizeof(Rating) == 2 * sizeof(unsigned int), "Rating must stay padding-free");

}
