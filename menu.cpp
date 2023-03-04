/*
 * author : Shuichi TAKANO
 * since  : Sat Mar 04 2023 19:44:25
 */

#include "menu.h"
#include "textplane.h"
#include "device_def.h"

namespace ui
{

    void
    Menu::appendItem(int *value,
                     const char *modeText,
                     const char *const *valuesBegin, const char *const *valuesEnd,
                     const char *buttonText,
                     UpdateFunc &&valueUpdateFunc, UpdateFunc &&buttonFunc)
    {
        Item item;
        item.value = value;
        item.modeText = modeText;
        item.valueTexts = valuesBegin;
        item.range[0] = 0;
        item.range[1] = valuesEnd - valuesBegin - 1;
        item.loop = true;
        item.buttonText = buttonText;
        item.valueUpdateFunc = std::move(valueUpdateFunc);
        item.buttonFunc = std::move(buttonFunc);
        items_.push_back(std::move(item));
    }

    void
    Menu::appendItem(int *value, const std::array<int, 2> &range,
                     const char *modeText,
                     const char *buttonText,
                     UpdateFunc &&valueUpdateFunc, UpdateFunc &&buttonFunc)
    {
        Item item;
        item.value = value;
        item.range = range;
        item.modeText = modeText;
        item.loop = false;
        item.buttonText = buttonText;
        item.valueUpdateFunc = std::move(valueUpdateFunc);
        item.buttonFunc = std::move(buttonFunc);
        items_.push_back(std::move(item));
    }

    void
    Menu::appendItem(const char *modeText, const char *buttonText, UpdateFunc &&buttonFunc)
    {
        Item item;
        item.modeText = modeText;
        item.buttonText = buttonText;
        item.buttonFunc = std::move(buttonFunc);
        items_.push_back(std::move(item));
    }

    void
    Menu::update(int buttons)
    {
        if (!isOpened())
        {
            if (!device::isAnyButtonPushed(prevButtons_) && device::isAnyButtonPushed(buttons))
            {
                open();
            }
            else
            {
                prevButtons_ = buttons;
                return;
            }
        }

        ++frameCounter_;

        const auto &item = items_[currentItem_];

        if (device::isButtonEdge(buttons, prevButtons_, device::Button::UP))
        {
            --currentItem_;
            if (currentItem_ < 0)
            {
                currentItem_ = static_cast<int>(items_.size() - 1);
            }
            frameCounter_ = 0;
        }
        else if (device::isButtonEdge(buttons, prevButtons_, device::Button::DOWN))
        {
            ++currentItem_;
            if (currentItem_ >= static_cast<int>(items_.size()))
            {
                currentItem_ = 0;
            }
            frameCounter_ = 0;
        }
        else if (device::isButtonEdge(buttons, prevButtons_, device::Button::CENTER))
        {
            if (item.buttonFunc)
            {
                item.buttonFunc();
            }
            frameCounter_ = 0;
        }
        else if (device::isButtonEdge(buttons, prevButtons_, device::Button::LEFT))
        {
            if (item.value)
            {
                if (*item.value > item.range[0])
                {
                    --*item.value;
                }
                else if (item.loop)
                {
                    *item.value = item.range[1];
                }
            }
            if (item.valueUpdateFunc)
            {
                item.valueUpdateFunc();
            }
            frameCounter_ = 0;
        }
        else if (device::isButtonEdge(buttons, prevButtons_, device::Button::RIGHT))
        {
            if (item.value)
            {
                if (*item.value < item.range[1])
                {
                    ++*item.value;
                }
                else if (item.loop)
                {
                    *item.value = item.range[0];
                }
            }

            if (item.valueUpdateFunc)
            {
                item.valueUpdateFunc();
            }
            frameCounter_ = 0;
        }
        else if (frameCounter_ > 300)
        {
            close();
        }

        prevButtons_ = buttons;
    }

    void
    Menu::open()
    {
        currentItem_ = 0;
        frameCounter_ = 0;
    }

    void
    Menu::close()
    {
        currentItem_ = -1;
    }

    void
    Menu::render(graphics::TextPlane &textPlane) const
    {
        if (!isOpened())
        {
            return;
        }

        textPlane.enableShadow(true);
        textPlane.enableTransBlack(true);
        textPlane.setColor(7);
        textPlane.setShadowColor(0);

        constexpr char LEFT = 1;
        constexpr char RIGHT = 2;
        constexpr char UP = 3;
        constexpr char DOWN = 4;
        constexpr char BUTTON = 5;

        constexpr size_t x = 10;
        constexpr size_t y = 20;
        constexpr size_t w = 29;
        constexpr size_t h = 4;

        textPlane.fillBlack(x, y, w, h);

        bool blink = (frameCounter_ >> 4) & 1;
        const auto &item = items_[currentItem_];
        if (item.value)
        {
            const auto bl = blink ? ' ' : LEFT;
            const auto br = blink ? ' ' : RIGHT;

            if (item.valueTexts)
            {
                textPlane.printf(x + 1, y + 1, "%-10s : %c %-10s %c",
                                 item.modeText,
                                 bl,
                                 item.valueTexts[*item.value],
                                 br);
            }
            else
            {
                textPlane.printf(x + 1, y + 1, "%-10s : %c    %4d    %c",
                                 item.modeText,
                                 bl,
                                 *item.value,
                                 br);
            }
        }
        else
        {
            textPlane.printf(x + 1, y + 1, "%s", item.modeText);
        }

        if (!blink)
        {
            textPlane.printf(x + 6, y + 0, "%c", UP);
            textPlane.printf(x + 6, y + 2, "%c", DOWN);
        }

        if (item.buttonText)
        {
            textPlane.setColor(6);
            textPlane.printf(x + 11, y + 2, "%c %s", blink ? ' ' : BUTTON, item.buttonText);
        }
    }
}
