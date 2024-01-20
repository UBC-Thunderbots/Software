#include "play_intent.h"


PlayIntent::PlayIntent(IntentType play_intent_type)
    : intent(play_intent_type)
{
}

IntentType PlayIntent::getIntentType() const
{
    return intent;
}
