module Kinematics
  module Delta
    class Solver
      SQRT_3  = Math.sqrt(3)
      SIN_120 = Math.sqrt(3)/2
      COS_120 = -0.5
      TAN_60  = Math.sqrt(3)
      SIN_30  = 0.5
      TAN_30  = 1/Math.sqrt(3)

      def initialize(e, f, re, rf)
        @e  = e
        @f  = f
        @re = re
        @rf = rf
      end

      def forward(theta1, theta2, theta3)
        t = (@f-@e)*TAN_30/2
        dtr = Math::PI/180.0

        theta1 *= dtr
        theta2 *= dtr
        theta3 *= dtr

        y1 = -(t + @rf*Math.cos(theta1))
        z1 = -@rf*Math.sin(theta1)

        y2 = (t + @rf*Math.cos(theta2))*SIN_30
        x2 = y2*TAN_60
        z2 = -@rf*Math.sin(theta2)

        y3 = (t + @rf*Math.cos(theta3))*SIN_30
        x3 = -y3*TAN_60
        z3 = -@rf*Math.sin(theta3)

        dnm = (y2-y1)*x3-(y3-y1)*x2

        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3

        # x = (a1*z + b1)/dnm
        a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
        b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0

        # y = (a2*z + b2)/dnm
        a2 = -(z2-z1)*x3+(z3-z1)*x2
        b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0

        # a*z^2 + b*z + c = 0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
        c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - @re*@re)

        # discriminant
        d = b*b - 4.0*a*c
        raise if d < 0

        z0 = -0.5*(b+Math.sqrt(d))/a
        x0 = (a1*z0 + b1)/dnm
        y0 = (a2*z0 + b2)/dnm

        [x0.round(3), y0.round(3), z0.round(3)]
      end

      def calc_angle_yz(x0, y0, z0)
        y1 = -0.5 * 0.57735 * @f; # f/2 * tg 30
        y0 -= 0.5 * 0.57735 * @e;    # shift center to edge
        # z = a + b*y
        a = (x0*x0 + y0*y0 + z0*z0 +@rf*@rf - @re*@re - y1*y1)/(2*z0);
        b = (y1-y0)/z0;
        # discriminant
        d = -(a+b*y1)*(a+b*y1)+@rf*(b*b*@rf+@rf); 
        raise if d < 0

        yj = (y1 - a*b - Math.sqrt(d))/(b*b + 1); # choosing outer point
        zj = a + b*yj;
        theta = 180.0*Math.atan(-zj/(y1 - yj))/Math::PI + ((yj>y1)?180.0:0.0);
      end

      # inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
      # returned status: 0=OK, -1=non-existing position
      def inverse(x0, y0, z0)
       theta1 = theta2 = theta3 = 0.0;
       theta_1 = calc_angle_yz(x0, y0, z0)
       theta_2 = calc_angle_yz(x0*COS_120 + y0*SIN_120, y0*COS_120-x0*SIN_120, z0);  # rotate coords to +120 deg
       theta_3 = calc_angle_yz(x0*COS_120 - y0*SIN_120, y0*COS_120+x0*SIN_120, z0);  # rotate coords to -120 deg

       [theta_1.round(3), theta_2.round(3), theta_3.round(3)]
     end
    end
  end
end
