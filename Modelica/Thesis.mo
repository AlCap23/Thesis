package Thesis "Package for the Master Thesis"
  partial model MiMoSystem " Multiple Input, Multiple Output Model"
    parameter Real num11 "numerator of G11";
    parameter Real den11 "denominator of G11";
    parameter Real y_start11 "start output of G11";
    parameter Real num22;
    parameter Real den22;
    parameter Real y_start22;
    parameter Real num21;
    parameter Real den21;
    parameter Real y_start21;
    parameter Real num12;
    parameter Real den12;
    parameter Real y_start12;
    Modelica.Blocks.Continuous.TransferFunction G11(a = den11, b = num11, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = y_start11) annotation(
      Placement(visible = true, transformation(origin = {-52, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction G12(a = den12, b = num12, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = y_start12) annotation(
      Placement(visible = true, transformation(origin = {-52, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction G21(a = den21, b = num21, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = y_start21) annotation(
      Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction G22(a = den22, b = num22, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = y_start22) annotation(
      Placement(visible = true, transformation(origin = {-52, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add_1 annotation(
      Placement(visible = true, transformation(origin = {12, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(visible = true, transformation(origin = {6, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u_1 annotation(
      Placement(visible = true, transformation(origin = {-114, 66}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u_2 annotation(
      Placement(visible = true, transformation(origin = {-124, -22}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -72}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y_1 annotation(
      Placement(visible = true, transformation(origin = {58, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y_2 annotation(
      Placement(visible = true, transformation(origin = {108, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(u_2, G22.u) annotation(
      Line(points = {{-126, -22}, {-64, -22}, {-64, -32}}, color = {0, 0, 127}));
    connect(u_2, G21.u) annotation(
      Line(points = {{-126, -22}, {-66, -22}, {-66, 0}, {-64, 0}}, color = {0, 0, 127}));
    connect(add1.y, y_2) annotation(
      Line(points = {{18, -26}, {106, -26}}, color = {0, 0, 127}));
    connect(add_1.y, y_1) annotation(
      Line(points = {{24, 60}, {50, 60}, {50, 60}, {58, 60}}, color = {0, 0, 127}));
    connect(G12.y, add1.u1) annotation(
      Line(points = {{-40, 38}, {-8, 38}, {-8, -20}, {-6, -20}}, color = {0, 0, 127}));
    connect(G21.y, add_1.u2) annotation(
      Line(points = {{-40, 0}, {-2, 0}, {-2, 54}, {0, 54}}, color = {0, 0, 127}));
    connect(G12.u, u_1) annotation(
      Line(points = {{-64, 38}, {-106, 38}, {-106, 66}, {-114, 66}}, color = {0, 0, 127}));
    connect(u_1, G11.u) annotation(
      Line(points = {{-114, 66}, {-64, 66}, {-64, 66}, {-64, 66}}, color = {0, 0, 127}));
    connect(add1.u2, G22.y) annotation(
      Line(points = {{-6, -32}, {-40, -32}}, color = {0, 0, 127}));
    connect(G11.y, add_1.u1) annotation(
      Line(points = {{-40, 66}, {0, 66}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(initialScale = 0.7), graphics = {Text(origin = {-7, 5}, extent = {{-61, 67}, {61, -67}}, textString = "G", textStyle = {TextStyle.Bold, TextStyle.UnderLine}), Rectangle(origin = {-2, 1}, extent = {{-90, 93}, {90, -93}})}),
      Documentation(info = "<html><head></head><body><div>Realizes</div><div><br></div>Y = G(s) U<div><br></div><div>in the Time Domain<br><div><br></div><div>Whereas dim(Y) = dim(U) = 2 and dim(G(s)) = dim(Y) x dim(U).</div><div><br></div><div>Takes as modifiers numXY, denXY and y_startXY, X=Y from 1 to 2.</div><div>Sets the Parameters of the single Transfer Function GXY from Input X to Output Y.</div><div><br></div></div></body></html>"));
  end MiMoSystem;

  model Test
    extends MiMo;
  end Test;

  partial model MiMoControl
    parameter Real K11 "Gain of F11";
    parameter Real T11 "Time Constant of F11";
    parameter Real y_start11 "Initial Output of F11";
    parameter Real K12;
    parameter Real T12;
    parameter Real y_start12;
    parameter Real K21;
    parameter Real T21;
    parameter Real y_start21;
    parameter Real K22;
    parameter Real T22;
    parameter Real y_start22;
    Modelica.Blocks.Continuous.PI C11(T = T11, k = K11, y_start = y_start11) annotation(
      Placement(visible = true, transformation(origin = {-50, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI C12(T = T12, k = K12, y_start = y_start12) annotation(
      Placement(visible = true, transformation(origin = {-52, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI C21(T = T21, k = K21, y_start = y_start21) annotation(
      Placement(visible = true, transformation(origin = {-52, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI C22(T = T22, k = K22, y_start = y_start22) annotation(
      Placement(visible = true, transformation(origin = {-52, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(visible = true, transformation(origin = {40, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add2 annotation(
      Placement(visible = true, transformation(origin = {32, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u_1 annotation(
      Placement(visible = true, transformation(origin = {-114, 56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 68}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u_2 annotation(
      Placement(visible = true, transformation(origin = {-116, -44}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y_1 annotation(
      Placement(visible = true, transformation(origin = {100, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y_2 annotation(
      Placement(visible = true, transformation(origin = {108, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(C11.u, u_1) annotation(
      Line(points = {{-62, 64}, {-104, 64}, {-104, 56}, {-114, 56}}, color = {0, 0, 127}));
    connect(C12.u, u_1) annotation(
      Line(points = {{-64, 32}, {-86, 32}, {-86, 56}, {-114, 56}, {-114, 56}}, color = {0, 0, 127}));
    connect(C21.u, u_2) annotation(
      Line(points = {{-64, -6}, {-106, -6}, {-106, -44}, {-116, -44}}, color = {0, 0, 127}));
    connect(C22.u, u_2) annotation(
      Line(points = {{-64, -46}, {-104, -46}, {-104, -44}, {-116, -44}}, color = {0, 0, 127}));
    connect(add2.y, y_2) annotation(
      Line(points = {{44, -40}, {70, -40}, {70, -26}, {108, -26}, {108, -26}}, color = {0, 0, 127}));
    connect(add1.y, y_1) annotation(
      Line(points = {{52, 58}, {86, 58}, {86, 58}, {100, 58}, {100, 58}}, color = {0, 0, 127}));
    connect(C21.y, add1.u2) annotation(
      Line(points = {{-40, -6}, {28, -6}, {28, 52}}, color = {0, 0, 127}));
    connect(C11.y, add1.u1) annotation(
      Line(points = {{-38, 64}, {28, 64}}, color = {0, 0, 127}));
    connect(C22.y, add2.u2) annotation(
      Line(points = {{-40, -46}, {20, -46}}, color = {0, 0, 127}));
    connect(C12.y, add2.u1) annotation(
      Line(points = {{-40, 32}, {20, 32}, {20, -34}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(initialScale = 0.7), graphics = {Text(origin = {4, -1}, extent = {{-92, 43}, {92, -43}}, textString = "P", textStyle = {TextStyle.Bold, TextStyle.UnderLine}), Rectangle(origin = {0, 1}, extent = {{-88, 93}, {88, -93}})}));
  end MiMoControl;

  model Test_SmallCoupling
    replaceable Thesis.MiMoSystem Plant(den11 = {5, 1}, den12 = {30, 1}, den21 = {50, 1}, den22 = {10, 1}, num11 = {2}, num12 = {10}, num21 = {1}, num22 = {5}, y_start11 = 1, y_start12 = 1, y_start21 = 1, y_start22 = 2) annotation(
      Placement(visible = true, transformation(origin = {9, 37}, extent = {{-27, -27}, {27, 27}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = 5, offset = 1, startTime = 2) annotation(
      Placement(visible = true, transformation(origin = {-78, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(Plant.u_2, step1.y) annotation(
      Line(points = {{-12, 18}, {-66, 18}, {-66, 38}, {-66, 38}}, color = {0, 0, 127}));
    connect(Plant.u_1, step1.y) annotation(
      Line(points = {{-12, 56}, {-66, 56}, {-66, 38}, {-66, 38}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(initialScale = 0.7)));
  end Test_SmallCoupling;

  model WeakCoupling
    extends Thesis.MiMoSystem;
    annotation(
      Icon(coordinateSystem(initialScale = 0.7)));
  end WeakCoupling;
  annotation(
    uses(Modelica(version = "3.2.2")));
end Thesis;