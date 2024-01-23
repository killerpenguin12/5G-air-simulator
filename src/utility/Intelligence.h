#ifndef INTELLIGENCE_H_
#define INTELLIGENCE_H_

namespace Intel
{
    // Define a class
    class Intelligence
    {
    public:
        Intelligence(int val)
            : value_(val)
        {}

        int get_value() const
        {
            return value_;
        }

        void set_value(const int val)
        {
            value_ = val;
        }

    private:
        int value_;
    };

    // Declare a function prototype
    void bar(Intelligence& v);
}

#endif /* INTELLIGENCE_H_ */