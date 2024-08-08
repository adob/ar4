#include "tts.h"

#include <QTextToSpeech>
#include "lib/sync.h"

using namespace lib;

static QTextToSpeech *speech;
static sync::Mutex mutex;

void tts::say(str s) {
    sync::Lock lock(mutex);

    if (!speech) {
        speech = new QTextToSpeech();
    }
    speech->say(QString::fromUtf8(s.data, s.len));
}