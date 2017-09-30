within ;
package FMU_Physical "Package for Physical Systems"
  package HP_HT_Control
    "Package for controlling the high pressure and temperature"

    model Identification "Physical System"
      Supermarktmodelle_342.Identification_Physical_System.Inputs
        Physical_System(pHochdruckStart=80e5)
        annotation (Placement(transformation(extent={{-30,-18},{34,22}})));
      Modelica.Blocks.Interfaces.RealOutput T_H
        annotation (Placement(transformation(extent={{96,6},{116,26}})));
      Modelica.Blocks.Interfaces.RealOutput p_H
        annotation (Placement(transformation(extent={{96,-14},{116,6}})));
      Modelica.Blocks.Sources.RealExpression T_A(y=300)
        "Ambient Temperature"
        annotation (Placement(transformation(extent={{-100,6},{-80,26}})));
      Modelica.Blocks.Sources.RealExpression P_M(y=35e5) "Medium Pressure"
        annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      Modelica.Blocks.Sources.RealExpression Q_C(y=-60e3)
        "Heat flow into the system"
        annotation (Placement(transformation(extent={{-100,-26},{-80,-6}})));
      Masterthesis.Controller.Multivariable_Controller multivariable_Controller(
        n=2,
        KP={{-0.1,0.},{0.0,-0.003}},
        KI=[-0.1/480.,0.0; 0.0,-0.003/100.],
        ymax={20,7},
        ymin={5,2},
        activationTime=200,
        offset={20,5.5},
        use_activeInput=true,
        B={{1,0},{0,1}},
        D={{1,0},{0,1}})      annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-10,70})));
      Modelica.Blocks.Math.Add add_T annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,74})));
      Modelica.Blocks.Math.Add add_P annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,44})));
      Modelica.Blocks.Sources.Step step_T(
        offset=0,
        height=-1,
        startTime=1e5)
        annotation (Placement(transformation(extent={{-142,68},{-122,88}})));
      Modelica.Blocks.Sources.Step step_P(
        height=1,
        offset=0,
        startTime=50000)
        annotation (Placement(transformation(extent={{-144,32},{-124,52}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=true,
          startTime=50000)
        annotation (Placement(transformation(extent={{-2,34},{18,54}})));
      Modelica.Blocks.Sources.RealExpression T_R(y=T_A.y + 3) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={66,78})));
      Modelica.Blocks.Sources.RealExpression P_R(y=max(50, 1.62124822*T_A.y -
            401.68)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={66,60})));
    equation
      connect(Physical_System.yTGC, T_H) annotation (Line(points={{34.8,16.8},{
              58.4,16.8},{58.4,16},{106,16}}, color={0,0,127}));
      connect(Physical_System.yHP, p_H) annotation (Line(points={{35,-4.4},{
              64.5,-4.4},{64.5,-4},{106,-4}}, color={0,0,127}));
      connect(Physical_System.TUmgebung, T_A.y) annotation (Line(points={{-30,
              16.8},{-54,16.8},{-54,16},{-79,16}}, color={0,0,127}));
      connect(Physical_System.Mitteldruck, P_M.y) annotation (Line(points={{-30,
              2.6},{-54,2.6},{-54,0},{-79,0}}, color={0,0,127}));
      connect(Physical_System.Qdotkaelte, Q_C.y)
        annotation (Line(points={{-30,-16},{-79,-16}}, color={0,0,127}));
      connect(Physical_System.yTGC, multivariable_Controller.u_m[1])
        annotation (Line(points={{34.8,16.8},{46,16.8},{46,94},{18,94},{-6.2,94},
              {-6.2,88},{-6.2,81.9}}, color={0,0,127}));
      connect(Physical_System.yHP, multivariable_Controller.u_m[2]) annotation (
         Line(points={{35,-4.4},{38,-4.4},{38,94},{24,94},{-6.2,94},{-6.2,86},{
              -6.2,79.9}}, color={0,0,127}));
      connect(add_T.u2, multivariable_Controller.y[1]) annotation (Line(points=
              {{-62,80},{-52,80},{-52,78},{-38,78},{-38,70.5},{-20.6,70.5}},
            color={0,0,127}));
      connect(add_P.u2, multivariable_Controller.y[2]) annotation (Line(points=
              {{-62,50},{-52,50},{-52,52},{-30,52},{-30,69.5},{-20.6,69.5}},
            color={0,0,127}));
      connect(add_T.y, Physical_System.nFan) annotation (Line(points={{-85,74},
              {-92,74},{-92,30},{-15,30},{-15,22}}, color={0,0,127}));
      connect(add_P.y, Physical_System.Ventil) annotation (Line(points={{-85,44},
              {-88,44},{-88,28},{7.8,28},{7.8,22}}, color={0,0,127}));
      connect(step_T.y, add_T.u1) annotation (Line(points={{-121,78},{-88,78},{
              -88,70},{-54,70},{-54,68},{-62,68}}, color={0,0,127}));
      connect(step_P.y, add_P.u1) annotation (Line(points={{-123,42},{-86,42},{
              -50,42},{-50,38},{-62,38}}, color={0,0,127}));
      connect(booleanStep.y, multivariable_Controller.activeInput) annotation (
          Line(points={{19,44},{19,44},{-10,44},{-10,59}}, color={255,0,255}));
      connect(T_R.y, multivariable_Controller.u_s[1]) annotation (Line(points={
              {55,78},{1.77636e-015,78},{1.77636e-015,79}}, color={0,0,127}));
      connect(P_R.y, multivariable_Controller.u_s[2]) annotation (Line(points={
              {55,60},{34,60},{34,70},{8.88178e-016,70},{8.88178e-016,77}},
            color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Identification;

    model Identification_ExternalInput "Physical System"
      Supermarktmodelle_342.Identification_Physical_System.Inputs
        Physical_System(pHochdruckStart=80e5)
        annotation (Placement(transformation(extent={{-30,-18},{34,22}})));
      Modelica.Blocks.Interfaces.RealOutput T_H
        annotation (Placement(transformation(extent={{96,6},{116,26}})));
      Modelica.Blocks.Interfaces.RealOutput p_H
        annotation (Placement(transformation(extent={{96,-14},{116,6}})));
      Modelica.Blocks.Sources.RealExpression P_M(y=35e5) "Medium Pressure"
        annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      Masterthesis.Controller.Multivariable_Controller multivariable_Controller(
        n=2,
        KP={{-0.1,0.},{0.0,-0.003}},
        KI=[-0.1/480.,0.0; 0.0,-0.003/100.],
        ymax={20,7},
        ymin={5,2},
        activationTime=200,
        offset={20,5.5},
        use_activeInput=true,
        B={{1,0},{0,1}},
        D={{1,0},{0,1}})      annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-10,70})));
      Modelica.Blocks.Math.Add add_T annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,74})));
      Modelica.Blocks.Math.Add add_P annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-74,44})));
      Modelica.Blocks.Sources.Step step_T(
        offset=0,
        height=-1,
        startTime=1e5)
        annotation (Placement(transformation(extent={{-142,68},{-122,88}})));
      Modelica.Blocks.Sources.Step step_P(
        height=1,
        offset=0,
        startTime=50000)
        annotation (Placement(transformation(extent={{-144,32},{-124,52}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=true,
          startTime=50000)
        annotation (Placement(transformation(extent={{-2,34},{18,54}})));
      Modelica.Blocks.Sources.RealExpression T_R(y=T_A + 3)   annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={68,78})));
      Modelica.Blocks.Sources.RealExpression P_R(y=max(50, 1.62124822*T_A -
            401.68)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={66,60})));
      Modelica.Blocks.Interfaces.RealInput T_A
        annotation (Placement(transformation(extent={{-132,-2},{-92,38}})));
      Modelica.Blocks.Interfaces.RealInput Q_C
        annotation (Placement(transformation(extent={{-134,-38},{-94,2}})));
    equation
      connect(Physical_System.yTGC, T_H) annotation (Line(points={{34.8,16.8},{
              58.4,16.8},{58.4,16},{106,16}}, color={0,0,127}));
      connect(Physical_System.yHP, p_H) annotation (Line(points={{35,-4.4},{
              64.5,-4.4},{64.5,-4},{106,-4}}, color={0,0,127}));
      connect(Physical_System.Mitteldruck, P_M.y) annotation (Line(points={{-30,
              2.6},{-54,2.6},{-54,0},{-79,0}}, color={0,0,127}));
      connect(Physical_System.yTGC, multivariable_Controller.u_m[1])
        annotation (Line(points={{34.8,16.8},{46,16.8},{46,94},{18,94},{-6.2,94},
              {-6.2,88},{-6.2,81.9}}, color={0,0,127}));
      connect(Physical_System.yHP, multivariable_Controller.u_m[2]) annotation (
         Line(points={{35,-4.4},{38,-4.4},{38,94},{24,94},{-6.2,94},{-6.2,86},{
              -6.2,79.9}}, color={0,0,127}));
      connect(add_T.u2, multivariable_Controller.y[1]) annotation (Line(points=
              {{-62,80},{-52,80},{-52,78},{-38,78},{-38,70.5},{-20.6,70.5}},
            color={0,0,127}));
      connect(add_P.u2, multivariable_Controller.y[2]) annotation (Line(points=
              {{-62,50},{-52,50},{-52,52},{-30,52},{-30,69.5},{-20.6,69.5}},
            color={0,0,127}));
      connect(add_T.y, Physical_System.nFan) annotation (Line(points={{-85,74},
              {-92,74},{-92,30},{-15,30},{-15,22}}, color={0,0,127}));
      connect(add_P.y, Physical_System.Ventil) annotation (Line(points={{-85,44},
              {-88,44},{-88,28},{7.8,28},{7.8,22}}, color={0,0,127}));
      connect(step_T.y, add_T.u1) annotation (Line(points={{-121,78},{-88,78},{
              -88,70},{-54,70},{-54,68},{-62,68}}, color={0,0,127}));
      connect(step_P.y, add_P.u1) annotation (Line(points={{-123,42},{-86,42},{
              -50,42},{-50,38},{-62,38}}, color={0,0,127}));
      connect(booleanStep.y, multivariable_Controller.activeInput) annotation (
          Line(points={{19,44},{19,44},{-10,44},{-10,59}}, color={255,0,255}));
      connect(T_R.y, multivariable_Controller.u_s[1]) annotation (Line(points={{57,78},
              {1.77636e-015,78},{1.77636e-015,79}},         color={0,0,127}));
      connect(P_R.y, multivariable_Controller.u_s[2]) annotation (Line(points={
              {55,60},{34,60},{34,70},{8.88178e-016,70},{8.88178e-016,77}},
            color={0,0,127}));
      connect(T_A, Physical_System.TUmgebung) annotation (Line(points={{-112,18},
              {-72,18},{-72,16.8},{-30,16.8}}, color={0,0,127}));
      connect(Q_C, Physical_System.Qdotkaelte) annotation (Line(points={{-114,
              -18},{-72,-18},{-72,-16},{-30,-16}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Identification_ExternalInput;
  end HP_HT_Control;
  annotation (uses(Supermarktmodelle_342(version="1"), Modelica(version="3.2.2")));
end FMU_Physical;
