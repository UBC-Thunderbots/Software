#include "software/proto/message_translation/ssl_message_translator.h"

#include <gtest/gtest.h>
#include "software/test_util/test_util.h"

TEST(SSLMessageTranslatorTest, test) {
    Field field = ::Test::TestUtil::createSSLDivBField();

    auto field_msg = createGeometryFieldSize(field);
    for(const auto& foo : field_msg->field_lines()) {
        std::cout << foo.name() << std::endl;
    }
//    std::cout << field_msg->kFieldLinesFieldNumber << std::endl;
}