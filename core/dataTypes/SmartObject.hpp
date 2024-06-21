//
// Created by masoud on 6/21/24.
//
// Note since it's a performance bottleneck to send/receive
// messages from modules (e.g. millions of map points),
// it is better to use flags to mark changes on distributed pointer
// objects, to be changed locally

#ifndef NAV24_SMARTOBJECT_HPP
#define NAV24_SMARTOBJECT_HPP


namespace NAV24 {
    class SmartObject {
    public:
        SmartObject() : mbValid(true), mbVisible(true) {}

        [[nodiscard]] virtual bool isValid() const {
            return mbValid;
        }

        virtual void setValid(bool bValid) {
            SmartObject::mbValid = bValid;
        }

        [[nodiscard]] virtual bool isVisible() const {
            return mbVisible;
        }

        virtual void setVisible(bool bVisible) {
            SmartObject::mbVisible = bVisible;
        }

    protected:
        bool mbValid;
        bool mbVisible;
    };
} // NAV24

#endif //NAV24_SMARTOBJECT_HPP
