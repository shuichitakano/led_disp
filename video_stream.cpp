/*
 * author : Shuichi TAKANO
 * since  : Sat Feb 11 2023 13:50:29
 */

#include "video_stream.h"

namespace video
{

    std::tuple<BT656TimingCode, bool>
    findNextEAV(const uint32_t *buffer, uint32_t activeWidthInWords)
    {
        auto eav = buffer[activeWidthInWords];
        bool error = false;

        // マーカーがずれていそうならエラーとしつつリカバリーを試みる
        if ((eav & 0xff) != 0xff)
        {
            error = true;
            auto pv = buffer[activeWidthInWords + 1];
            do
            {
                eav = (eav >> 8) | ((pv << 24) & 0xff000000);
                pv >>= 8;
            } while (pv && (eav & 0xff) != 0xff);
        }
        return {static_cast<BT656TimingCode>(eav), error};
    }

    BT656TimingCode getSAVCorrespondingToEAV(BT656TimingCode eav)
    {
        switch (eav)
        {
        case BT656TimingCode::EAV_ACTIVE_F0:
            return BT656TimingCode::SAV_ACTIVE_F0;

        case BT656TimingCode::EAV_VSYNC_F0:
            return BT656TimingCode::SAV_VSYNC_F0;

        case BT656TimingCode::EAV_ACTIVE_F1:
            return BT656TimingCode::SAV_ACTIVE_F1;

        case BT656TimingCode::EAV_VSYNC_F1:
            return BT656TimingCode::SAV_VSYNC_F1;
        }
        return BT656TimingCode::INVALID;
    }

}
