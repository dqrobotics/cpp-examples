//Contributed by @mmmarinho
//https://github.com/dqrobotics/cpp/issues/24

#include "test_issue_24.h"

#include<dqrobotics/DQ.h>
using namespace DQ_robotics;

void test_issue_24()
{
    //General dual quaternion
    VectorXd general_dq(8);
    general_dq << 1.,2.,3.,4.,5.,6.,7.,8.;
    DQ a(general_dq);
    DQ b(1.,2.,3.,4.,5.,6.,7.,8.);
    assert(a==b);

    //Pure dual quaternion
    VectorXd pure_dq(6);
    pure_dq << 1.,2.,3.,4.,5.,6.;
    a = DQ(pure_dq);
    b = DQ(0.,1.,2.,3.,0.,4.,5.,6.);
    assert(a==b);

    //General quaternion
    VectorXd general_q(4);
    general_q << 1.,2.,3.,4.;
    a = DQ(general_q);
    b = DQ(1.,2.,3.,4.,0.,0.,0.,0.);
    assert(a==b);

    //Pure quaternion
    VectorXd pure_q(3);
    pure_q << 1.,2.,3.;
    a = DQ(pure_q);
    b = DQ(0.,1.,2.,3.,0.,0.,0.,0.);
    assert(a==b);

    //Real number
    VectorXd real(1);
    real << 1.;
    a = DQ(real);
    b = DQ(1.,0.,0.,0.,0.,0.,0.,0.);
    assert(a==b);
}
