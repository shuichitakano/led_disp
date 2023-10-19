/*
 * author : Shuichi TAKANO
 * since  : Sat Mar 04 2023 18:43:26
 */
#pragma once

#include <cstdint>
#include <functional>
#include <vector>
#include <array>

namespace graphics
{
    class TextPlane;
}

namespace ui
{
    class Menu
    {
    public:
        using UpdateFunc = std::function<void()>;
        using ConditionFunc = std::function<bool()>;

        struct Item
        {
            const char *modeText{};
            const char *buttonText{};

            int *value{};
            std::array<int, 2> range;
            bool loop = false;

            const char *const *valueTexts{};

            UpdateFunc valueUpdateFunc{};
            UpdateFunc buttonFunc{};
            ConditionFunc insensitiveFunc{};

            Item &setInsensitiveFunc(ConditionFunc &&f)
            {
                insensitiveFunc = std::move(f);
                return *this;
            }
        };

    public:
        void update(int buttons);

        Item &appendItem(int *value,
                         const char *modeText,
                         const char *const *valuesBegin, const char *const *valuesEnd,
                         const char *buttonText = {},
                         UpdateFunc &&valueUpdateFunc = {}, UpdateFunc &&buttonFunc = {});

        Item &appendItem(int *value, const std::array<int, 2> &range,
                         const char *modeText,
                         const char *buttonText = {},
                         UpdateFunc &&valueUpdateFunc = {}, UpdateFunc &&buttonFunc = {});

        void appendItem(const char *modeText, const char *buttonText, UpdateFunc &&buttonFunc);

        bool isOpened() const { return currentItem_ >= 0; }
        void open();
        void close();
        void render(graphics::TextPlane &textPlane) const;

        void setCloseFunc(std::function<void()> &&f) { closeFunc_ = std::move(f); }

    private:
        std::vector<Item> items_;

        std::function<void()> closeFunc_;

        uint16_t frameCounter_ = 0;

        int currentItem_ = -1;
        int prevButtons_ = 0;
    };
}
