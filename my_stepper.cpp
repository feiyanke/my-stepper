#include "my_stepper.h"

//每个dt调用,更新a,v,s, 如果需要输出step
void MyStepper::run()
{
    int16_t s = this->s;
    int16_t v = this->v;
    int16_t a = this->a;
    int16_t v1 = v + a;
    int16_t s1 = s + v;
    if(v>0)
    {
        if(s1<s)
        {
            this->stepP();
            this->step++;
        }
    }
    else
    {
        if(s1>s)
        {
            this->stepN();
            this->step--;
        }
    }
    this->s = s1;
    this->v = v1;
}
