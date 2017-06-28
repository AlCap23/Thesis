model Testing
  Modelica.Blocks.Continuous.Integrator integrator1 annotation(
    Placement(visible = true, transformation(origin = {-84, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 22, k = 5.5) annotation(
    Placement(visible = true, transformation(origin = {12, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(limitsAtInit = true, uMax = 10, uMin = 0) annotation(
    Placement(visible = true, transformation(origin = {-38, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.FixedDelay fixedDelay1(delayTime = 20.2) annotation(
    Placement(visible = true, transformation(origin = {72, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter2(limitsAtInit = true, uMax = 0.1) annotation(
    Placement(visible = true, transformation(origin = {-124, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-200, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 10) annotation(
    Placement(visible = true, transformation(origin = {-226, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 20, k = 5) annotation(
    Placement(visible = true, transformation(origin = {-40, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Nonlinear.FixedDelay fixedDelay2(delayTime = 20) annotation(
    Placement(visible = true, transformation(origin = {-86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Math.Feedback feedback2 annotation(
    Placement(visible = true, transformation(origin = {-86, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Math.Add add1 annotation(
    Placement(visible = true, transformation(origin = {-200, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Math.Feedback feedback3 annotation(
    Placement(visible = true, transformation(origin = {-164, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = 1) annotation(
    Placement(visible = true, transformation(origin = {-124, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
equation
  connect(add1.u2, firstOrder2.y) annotation(
    Line(points = {{-194, 10}, {-194, -2}, {-51, -2}}, color = {0, 0, 127}));
  connect(fixedDelay2.u, firstOrder2.y) annotation(
    Line(points = {{-86, -20}, {-86, -2}, {-51, -2}}, color = {0, 0, 127}));
  connect(firstOrder2.u, limiter1.y) annotation(
    Line(points = {{-28, -2}, {-14, -2}, {-14, 50}, {-26, 50}}, color = {0, 0, 127}));
  connect(feedback2.u2, fixedDelay2.y) annotation(
    Line(points = {{-86, -50}, {-86, -43}}, color = {0, 0, 127}));
  connect(gain1.u, limiter1.y) annotation(
    Line(points = {{-112, 18}, {-14, 18}, {-14, 50}, {-26, 50}, {-26, 50}}, color = {0, 0, 127}));
  connect(gain1.y, feedback3.u2) annotation(
    Line(points = {{-135, 18}, {-164, 18}, {-164, 42}}, color = {0, 0, 127}));
  connect(feedback3.y, limiter2.u) annotation(
    Line(points = {{-154, 48}, {-136, 48}, {-136, 50}, {-136, 50}}, color = {0, 0, 127}));
  connect(feedback3.u1, feedback1.y) annotation(
    Line(points = {{-172, 48}, {-190, 48}, {-190, 50}, {-190, 50}}, color = {0, 0, 127}));
  connect(add1.u1, feedback2.y) annotation(
    Line(points = {{-206, 10}, {-206, 10}, {-206, -58}, {-94, -58}, {-94, -58}}, color = {0, 0, 127}));
  connect(add1.y, feedback1.u2) annotation(
    Line(points = {{-200, 33}, {-200, 42}}, color = {0, 0, 127}));
  connect(feedback2.u1, fixedDelay1.y) annotation(
    Line(points = {{-78, -58}, {100, -58}, {100, 50}, {84, 50}, {84, 50}}, color = {0, 0, 127}));
  connect(firstOrder1.y, fixedDelay1.u) annotation(
    Line(points = {{24, 50}, {60, 50}}, color = {0, 0, 127}));
  connect(const.y, feedback1.u1) annotation(
    Line(points = {{-214, 50}, {-210, 50}, {-210, 50}, {-208, 50}}, color = {0, 0, 127}));
  connect(limiter1.y, firstOrder1.u) annotation(
    Line(points = {{-26, 50}, {-2, 50}, {-2, 50}, {0, 50}}, color = {0, 0, 127}));
  connect(integrator1.y, limiter1.u) annotation(
    Line(points = {{-72, 50}, {-52, 50}, {-52, 50}, {-50, 50}}, color = {0, 0, 127}));
  connect(limiter2.y, integrator1.u) annotation(
    Line(points = {{-112, 50}, {-98, 50}, {-98, 50}, {-96, 50}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(initialScale = 0.7)),
    uses(Modelica(version = "3.2.2")));
end Testing;