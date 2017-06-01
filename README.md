# MarkI-QuadCopter
Arduino Code of My First Drone Build with HC-05 and Android

This code is a derived version of Joop Brooking's Code(http://www.brokking.net/). 
It has been adapted to use blutetooth as a transiver, unlike traditinal RF.
Also, it Employs a 10DOF sensor, while the original used MPU6050.

Features:
  -Loop Time is well within 4ms. Hence a 250Hz cycle.
  -It's paired with a custom build Android App.
  -BodySync feature enables it to mimic phone's orientation.
  -Can easily be swapped from blutooth to wifi, if need be.


Input Fromat(HC-05):
  -All input comes in as String format
  -Throttle: "T1000", where "T" means a signal for throttle, followed by Value.
  -Start:"B"
  -Stop:"K"
  -Maneuver Signals: "SaaC", Where "S" refers to be a Manuever signal,
                      followed by Roll, Pitch and Yaw values.
                      Here "a" means -1, "b" means -2 and so on.
                      While "A" means 1, "B" means 2 and so on.
                      Hence the range is z-a-0-A-z (-27 to -1 to 0 to 1 to 27)
  -PID Calibration: "C0200", where "C" means signal for Calibration, "0" means P(of PID), followed by the Value*100
                    Hence, this says put P as (200/100) = 2
  
  -Reset Angles:"R", should be used to reset any manuever signal.
