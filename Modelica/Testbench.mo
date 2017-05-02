model Testbench
  Modelica.Blocks.Continuous.FirstOrder Plant_Main(T = 100, k = 5) annotation(
    Placement(visible = true, transformation(origin = {26, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder Plant_Coupling(T = 70, k = 2) annotation(
    Placement(visible = true, transformation(origin = {28, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1 annotation(
    Placement(visible = true, transformation(origin = {78, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI PI(T = 70, k = 2, y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {-56, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant w_1(k = 70) annotation(
    Placement(visible = true, transformation(origin = {-150, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant u_2(k = 30) annotation(
    Placement(visible = true, transformation(origin = {-10, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
    Placement(visible = true, transformation(origin = {-122, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k2 = -1) annotation(
    Placement(visible = true, transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(add2.u2, Plant_Coupling.y) annotation(
    Line(points = {{-102, 48}, {-114, 48}, {-114, 2}, {40, 2}, {40, 26}, {40, 26}}, color = {0, 0, 127}));
  connect(add2.y, PI.u) annotation(
    Line(points = {{-78, 54}, {-68, 54}, {-68, 60}, {-68, 60}}, color = {0, 0, 127}));
  connect(add2.u1, feedback1.y) annotation(
    Line(points = {{-102, 60}, {-112, 60}, {-112, 60}, {-112, 60}}, color = {0, 0, 127}));
  connect(add1.y, feedback1.u2) annotation(
    Line(points = {{90, 42}, {110, 42}, {110, -22}, {-122, -22}, {-122, 52}}, color = {0, 0, 127}));
  connect(w_1.y, feedback1.u1) annotation(
    Line(points = {{-138, 60}, {-130, 60}}, color = {0, 0, 127}));
  connect(u_2.y, Plant_Coupling.u) annotation(
    Line(points = {{1, 26}, {16, 26}}, color = {0, 0, 127}));
  connect(PI.y, Plant_Main.u) annotation(
    Line(points = {{-44, 60}, {14, 60}, {14, 60}, {14, 60}}, color = {0, 0, 127}));
  connect(Plant_Main.y, add1.u1) annotation(
    Line(points = {{37, 60}, {60, 60}, {60, 48}, {66, 48}}, color = {0, 0, 127}));
  connect(Plant_Coupling.y, add1.u2) annotation(
    Line(points = {{40, 26}, {60, 26}, {60, 36}, {66, 36}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(initialScale = 0.7)),
    uses(Modelica(version = "3.2.2")));
end Testbench;