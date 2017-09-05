package Masterthesis "Includes all necessary models"
  package Transferfunctions "Package for transfer functions."
    model miso_transferfunction
      "Creates an array of transfer function with multiple inputs and a single output"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer n "Number of inputs";
      parameter Real num[n, :] "Numerator of the system";
      parameter Real den[n, :] "Denominator of the system";
      parameter Real delay[n] " Delay of the system";
      Modelica.Blocks.Interfaces.RealInput u[n] "Connector of Real input signals" annotation (Placement(
            transformation(extent={{-120,-20},{-80,20}}), iconTransformation(
              extent={{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal" annotation (Placement(
            transformation(extent={{80,-20},{120,20}}), iconTransformation(extent=
               {{80,-20},{120,20}})));
      Modelica.Blocks.Continuous.TransferFunction[n] transferFunction( b = num,  a = den,
        each initType=Modelica.Blocks.Types.Init.InitialOutput)
        annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
      Modelica.Blocks.Math.MultiSum add(nu=n, significantDigits=5)
        annotation (Placement(transformation(extent={{24,-6},{36,6}})));

      Modelica.Blocks.Nonlinear.FixedDelay[n] fixedDelay(delayTime = delay)
        annotation (Placement(transformation(extent={{-24,-10},{-4,10}})));
    equation
      for inputs in 1:n loop
        connect(u[inputs],transferFunction[inputs].u);
        connect(transferFunction[inputs].y,fixedDelay[inputs].u);
        connect(fixedDelay[inputs].y,add.u[inputs]);
      end for;
      connect(add.y,y);
      annotation (Documentation(info="<html>
<p>
Block has a vector of continuous Real input signals and
one continuous Real output signal.
</p>
</html>"),   Icon(graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="g")}));
    end miso_transferfunction;

  model mimo_transferfunction "Makes a MIMO Transferfunction Matrix"
    extends Modelica.Blocks.Icons.Block;
    inner parameter Integer n  "Number of inputs (= number of outputs)";
    parameter Real num[n, n, :] "Numerator of the system";
    parameter Real den[n, n, :] "Denominator of the system";
    parameter Real delay[n,n] "Delay for the system";
    Modelica.Blocks.Interfaces.RealInput u[n] "Connector of Real input signals" annotation (
      Placement(transformation(extent={{-120,-20},{-80,20}}), iconTransformation(
              extent={{-120,-20},{-80,20}})));
    Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals" annotation (
      Placement(transformation(extent={{80,-20},{120,20}}), iconTransformation(
              extent={{80,-20},{120,20}})));

    Transferfunctions.miso_transferfunction[n] miso(
      each n=n,
      num=num,
      den=den,
      delay = delay) annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  equation
    for outputs in 1:n loop
      for inputs in 1:n loop
        connect(u[inputs],miso[outputs].u[inputs]);
        connect(miso[outputs].y,y[outputs]);
      end for;
    end for;
    annotation (
      Documentation(info = "<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"),   Icon(graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="G")}));
  end mimo_transferfunction;
  end Transferfunctions;















  package Controller "Package including controller, miso and mimo"
    model miso_decentralcontroller
      "Creates an array of PI controller with multiple inputs and a single output"
      extends Modelica.Blocks.Icons.Block;
      inner parameter Integer n "Number of inputs";
      parameter Real KP[n] "Proportional Gain";
      parameter Real KI[n] "Integral Gain";
      parameter Real B[n] "Set Point Weights";
      Modelica.Blocks.Interfaces.RealInput u_s[n] "Connector of Set Point value"
        annotation (Placement(transformation(extent={{-120,40},{-80,80}}),
            iconTransformation(extent={{-120,40},{-80,80}})));
      Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal" annotation (Placement(
            transformation(extent={{100,-8},{120,12}}), iconTransformation(extent=
               {{80,-20},{120,20}})));
      Modelica.Blocks.Math.MultiSum add(nu=n)
        annotation (Placement(transformation(extent={{24,-6},{36,6}})));
      PI_SP.PIController_sw[n] pIController_sw(
        each controllerType="PI",
        each integralParameterType="ki",
        k = KP,
        ki = KI,
        each use_setpointWeighting=true,
        b = B,
        each initType="initialOutput",
        each yMax=50,
        each yInitial=0)
        annotation (Placement(transformation(extent={{-50,-6},{-38,6}})));
      Modelica.Blocks.Interfaces.RealInput u_m[n]
        "Connector of measurement value" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-120,-60}), iconTransformation(extent={{-120,-80},{-80,-40}})));

    equation
      for inputs in 1:n loop
        connect(u_s[inputs],pIController_sw[inputs].u_s);
        connect(u_m[inputs],pIController_sw[inputs].u_m);
        connect(pIController_sw[inputs].y,add.u[inputs]);
      end for;
      connect(add.y,y);
      annotation (Documentation(info="<html>
<p>
Block has a vector of continuous Real input signals and
one continuous Real output signal.
</p>
</html>"),   Icon(graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="k")}));
    end miso_decentralcontroller;

    model mimo_decentralcontroller
      extends Modelica.Blocks.Icons.Block;
      parameter Integer n "Number of inputs (= number of outputs)";
      parameter Real KP[n,n] "Proportional Gain";
      parameter Real KI[n,n] "Integral Gain";
      parameter Real B[n,n] "Set Point Weights";
      Modelica.Blocks.Interfaces.RealInput u_s[n] "Connector of Set Point value" annotation (
        Placement(transformation(extent={{-120,40},{-80,80}}), iconTransformation(
              extent={{-120,40},{-80,80}})));
        Modelica.Blocks.Interfaces.RealInput u_m[n] "Connector of measurement value" annotation (
        Placement(transformation(extent={{-120,-80},{-80,-40}}),
            iconTransformation(extent={{-120,-80},{-80,-40}})));
      Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals" annotation (
        Placement(transformation(extent={{80,-20},{120,20}}), iconTransformation(
              extent={{80,-20},{120,20}})));

      Controller.miso_decentralcontroller[n] pi(
        each n=n,
        KP=KP,
        KI=KI,
        B=B) annotation (Placement(transformation(extent={{-14,-8},{6,12}})));
    equation
      for outputs in 1:n loop
        for inputs in 1:n loop
          connect(u_s[inputs],pi[outputs].u_s[inputs]);
          connect(u_m[inputs],pi[outputs].u_m[inputs]);
          connect(pi[outputs].y, y[outputs]);
        end for;
      end for;
      annotation (
        Documentation(info = "<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"),                 Icon(coordinateSystem(preserveAspectRatio=false),
            graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="K")}),                                              Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end mimo_decentralcontroller;

    model mimo_decoupler "Decoupler for MIMO Systems"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer n "Number of inputs";
      parameter Real[n,n] D "Decoupling Matrix";
      Modelica.Blocks.Interfaces.RealInput u[n] "Connector of Real input signals" annotation (Placement(
            transformation(extent={{-120,-20},{-80,20}}), iconTransformation(extent=
               {{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{80,-20},{120,20}}), iconTransformation(extent={{
                80,-20},{120,20}})));

    equation
      y = D*u; // Simple Matrix Multiplication

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
              extent={{-50,60},{42,-54}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="D")}),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end mimo_decoupler;
  end Controller;

  model Tester "Tester For All Kinds of Thing"

    Modelica.Blocks.Sources.Step step(
      height=1,
      offset=5,
      startTime=0)
      annotation (Placement(transformation(extent={{-88,74},{-68,94}})));
    Modelica.Blocks.Sources.Constant const(k=0)
      annotation (Placement(transformation(extent={{-96,-6},{-76,14}})));
    Controller.mimo_decentralcontroller mimo_decentralcontroller1(
      n=2,
      KP={{10,1},{1,5}},
      KI={{1,0},{-3,1}},
      B={{1,0},{0,1}})
      annotation (Placement(transformation(extent={{-28,26},{-8,46}})));
    Modelica.Blocks.Sources.Constant const1(k=0)
      annotation (Placement(transformation(extent={{-88,-38},{-68,-18}})));
    Modelica.Blocks.Sources.Step step1(
      height=1,
      offset=-3,
      startTime=10)
      annotation (Placement(transformation(extent={{-86,30},{-66,50}})));
  equation

    connect(mimo_decentralcontroller1.u_s[1], step.y) annotation (Line(points={
            {-30,40.2},{-48,40.2},{-48,84},{-67,84}}, color={0,0,127}));
    connect(mimo_decentralcontroller1.u_s[2], step1.y) annotation (Line(points=
            {{-30,42.2},{-48,42.2},{-48,40},{-65,40}}, color={0,0,127}));
    connect(mimo_decentralcontroller1.u_m[1], const.y) annotation (Line(points=
            {{-29.3,29.25},{-52,29.25},{-52,4},{-75,4}}, color={0,0,127}));
    connect(mimo_decentralcontroller1.u_m[2], const1.y) annotation (Line(points=
           {{-29.3,30.55},{-48,30.55},{-48,-28},{-67,-28}}, color={0,0,127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end Tester;

  package Testers "Tester for Package"

    model Test_MISOTF "Test for MISO Transferfunction"

      Transferfunctions.miso_transferfunction miso_transferfunction1(
        num={{10},{5},{-3}},
        den={{10,1},{5,1},{12,1}},
        n=3,
        delay={0,4,8})
        annotation (Placement(transformation(extent={{-12,-40},{8,-20}})));
      Modelica.Blocks.Sources.Step step(
        height=5,
        offset=1,
        startTime=2)
        annotation (Placement(transformation(extent={{-90,-2},{-70,18}})));
      Modelica.Blocks.Sources.Step step1(
        offset=1,
        height=-5,
        startTime=4)
        annotation (Placement(transformation(extent={{-90,-40},{-70,-20}})));
      Modelica.Blocks.Sources.Step step2(
        height=0.5,
        offset=-1,
        startTime=6)
        annotation (Placement(transformation(extent={{-90,-72},{-70,-52}})));
    equation
      connect(miso_transferfunction1.u[1], step.y) annotation (Line(points={{-12,
              -31.3333},{-42,-31.3333},{-42,8},{-69,8}},     color={0,0,127}));
      connect(miso_transferfunction1.u[2], step1.y) annotation (Line(points={{-12,-30},
              {-12,-30},{-69,-30}},          color={0,0,127}));
      connect(miso_transferfunction1.u[3], step2.y) annotation (Line(points={{-12,
              -28.6667},{-42,-28.6667},{-42,-62},{-69,-62}},     color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MISOTF;

    model Test_MIMOTF "Tester for MIMO System"

      Transferfunctions.mimo_transferfunction mimo_transferfunction1(
        n=3,
        num={{{3},{1e-10},{1}},{{4},{7},{1}},{{2},{5},{1e-10}}},
        den={{{10,1},{1e-3,1},{5,1}},{{4,6},{7,2},{1,1}},{{2,1},{50,1},{1e-3,1}}},
        delay={{10,0,0},{0,23,1},{0,0,5}})
        annotation (Placement(transformation(extent={{-8,-8},{12,12}})));
      Modelica.Blocks.Sources.Step step(
        height=-2,
        offset=1,
        startTime=0)
        annotation (Placement(transformation(extent={{-86,40},{-66,60}})));
      Modelica.Blocks.Sources.Step step1(
        height=3,
        offset=0,
        startTime=5)
        annotation (Placement(transformation(extent={{-86,-4},{-66,16}})));
      Modelica.Blocks.Sources.Step step2(
        height=-5,
        offset=0,
        startTime=10)
        annotation (Placement(transformation(extent={{-86,-40},{-66,-20}})));
    equation
      connect(mimo_transferfunction1.u[1], step.y) annotation (Line(points={{-8,
              0.666667},{-38,0.666667},{-38,50},{-65,50}},   color={0,0,127}));
      connect(mimo_transferfunction1.u[2], step1.y) annotation (Line(points={{-8,2},{
              -38,2},{-38,6},{-65,6}},             color={0,0,127}));
      connect(mimo_transferfunction1.u[3], step2.y) annotation (Line(points={{-8,
              3.33333},{-38,3.33333},{-38,-30},{-65,-30}},     color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MIMOTF;

    model Test_MISOCN "Tester for MISO controller"
      Controller.miso_decentralcontroller miso_decentralcontroller1(
        nin=2,
        KP={1,3},
        B={0.5,1},
        KI={0.02,0.01})
        annotation (Placement(transformation(extent={{-8,26},{12,46}})));
      Modelica.Blocks.Sources.Step step(
        height=1,
        offset=0,
        startTime=1)
        annotation (Placement(transformation(extent={{-92,74},{-72,94}})));
      Modelica.Blocks.Sources.Step step1(
        height=-3,
        offset=0,
        startTime=5)
        annotation (Placement(transformation(extent={{-88,2},{-68,22}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=1,
        duration=10,
        startTime=3)
        annotation (Placement(transformation(extent={{-88,30},{-68,50}})));
      Modelica.Blocks.Sources.Constant const
        annotation (Placement(transformation(extent={{-84,-28},{-64,-8}})));
    equation
      connect(miso_decentralcontroller1.u_m[2], step1.y) annotation (Line(
            points={{-8,30},{-38,30},{-38,12},{-67,12}},  color={0,0,127}));
      connect(miso_decentralcontroller1.u_m[1], const.y) annotation (Line(
            points={{-8,30},{-36,30},{-36,-18},{-63,-18}},  color={0,0,127}));
      connect(miso_decentralcontroller1.u_s[1], ramp.y) annotation (Line(points={{-8,42},
              {-38,42},{-38,40},{-67,40}},          color={0,0,127}));
      connect(miso_decentralcontroller1.u_s[2], step.y) annotation (Line(points={{-8,42},
              {-40,42},{-40,84},{-71,84}},          color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MISOCN;

    model Test_MIMOCN "Tester for MIMO Controller"
      Controller.mimo_decentralcontroller mimo_decentralcontroller1(
        n=2,
        KP={{10,1},{1,5}},
        KI={{0.1,0},{0.05,6}},
        B={{0,1},{0.2,0.3}})
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-90,20},{-70,40}})));
      Modelica.Blocks.Sources.Step step1(height=5, startTime=5)
        annotation (Placement(transformation(extent={{-88,-16},{-68,4}})));
      Modelica.Blocks.Sources.Step step2(height=-1, startTime=3)
        annotation (Placement(transformation(extent={{-88,-54},{-68,-34}})));
      Modelica.Blocks.Sources.Step step3(height=-1, startTime=3)
        annotation (Placement(transformation(extent={{-70,-90},{-50,-70}})));
    equation
      connect(step.y, mimo_decentralcontroller1.u_s[1]) annotation (Line(points={{-69,30},
              {-42,30},{-42,5},{-12,5}},                color={0,0,127}));
      connect(mimo_decentralcontroller1.u_s[2], step1.y) annotation (Line(
            points={{-12,7},{-40,7},{-40,-6},{-67,-6}},       color={0,0,127}));
      connect(mimo_decentralcontroller1.u_m[1], step2.y) annotation (Line(
            points={{-12,-7},{-40,-7},{-40,-44},{-67,-44}},         color={0,0,
              127}));
      connect(mimo_decentralcontroller1.u_m[2], step3.y) annotation (Line(
            points={{-12,-5},{-32,-5},{-32,-80},{-49,-80}},         color={0,0,
              127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MIMOCN;

    model Test_FMU "Tester for FMU"
      Masterthesis_Models_mimo_0processmodel_fmu
        masterthesis_Models_mimo_0processmodel_fmu(
        'num[1,2,1]'=0,
        'den[1,1,1]'=10,
        'den[1,1,2]'=1,
        'den[1,2,2]'=1,
        'den[2,1,2]'=1,
        'den[2,2,2]'=1,
        'num[1,1,1]'=10,
        'num[2,1,1]'=1,
        'num[2,2,1]'=5,
        'den[1,2,1]'=1,
        'den[2,1,1]'=5,
        'den[2,2,1]'=12)
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-82,8},{-62,28}})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
    equation
      connect(masterthesis_Models_mimo_0processmodel_fmu.u_1_, step.y)
        annotation (Line(points={{-12.4,3.4},{-36,3.4},{-36,18},{-61,18}},
            color={0,0,127}));
      connect(masterthesis_Models_mimo_0processmodel_fmu.u_2_, const.y)
        annotation (Line(points={{-12.4,-3.3},{-38,-3.3},{-38,-24},{-65,-24}},
            color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_FMU;
  end Testers;

  package Models "Package for used models"
    model mimo_closedloop

      parameter Integer n=2 "Number of in- and outputs";
      parameter Integer o=2 "Order of the Process";
      parameter Real[n,n,:] num = fill(1e-10,n,n,1) "Numerator of the system";
      parameter Real[n,n,:] den = fill(1e-10,n,n,o) "Denominator of the system";
      parameter Real[n,n] delay = zeros(n,n) "Delay of the system for every transfer function";
      parameter Real[n,n] kp = fill(1e-10,n,n) "Proportional gain of the controller";
      parameter Real[n,n] ki = fill(1e-10,n,n) "Integral gain of the controller";
      parameter Real[n,n] b =  fill(1e-10,n,n) "Set Point Weight of the controller";
      parameter Real[n,n] d =  fill(1e-10,n,n) "Decoupler of the System";
      Transferfunctions.mimo_transferfunction System(
        n=n,
        num=num,
        den=den,
        delay = delay) annotation (Placement(transformation(extent={{42,0},{62,20}})));
      Controller.mimo_decentralcontroller Decentral_Controller(
        n=n,
        B=b,
        KI=ki,
        KP=kp) annotation (Placement(transformation(extent={{-36,0},{-16,20}})));

      Controller.mimo_decoupler Decoupler(n=n, D=d)
        annotation (Placement(transformation(extent={{2,0},{22,20}})));
        Modelica.Blocks.Interfaces.RealInput u[n](each start=0) "Systems set point" annotation (
        Placement(transformation(extent={{-127,-18},{-92,18}}), iconTransformation(
              extent={{-120,-4},{-80,36}})));
           Modelica.Blocks.Interfaces.RealOutput y[n](each start=0) "Systems real output" annotation (
        Placement(transformation(extent={{92,-18},{128,18}}), iconTransformation(
              extent={{-120,40},{-80,80}})));
    equation
      for inputs in 1:n loop
        connect(u[inputs],Decentral_Controller.u_s[inputs]);
        connect(y[inputs],System.y[inputs]);
      end for;
      connect(Decentral_Controller.y, Decoupler.u)
        annotation (Line(points={{-16,10},{-7,10},{2,10}}, color={0,0,127}));
      connect(Decoupler.y, System.u)
        annotation (Line(points={{22,10},{32,10},{42,10}}, color={0,0,127}));
      connect(System.y, Decentral_Controller.u_m) annotation (Line(points={{62,10},{
              72,10},{80,10},{80,-20},{-50,-20},{-50,4},{-36,4}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end mimo_closedloop;

    model mimo_processmodel
      "Processmodel for a multiple input, multiple output system"

      parameter Integer n=2 "Number of in- and outputs";
      parameter Integer o=2 "Order of the Process";
      parameter Real[n,n,:] num = fill(1e-10,n,n,1) "Numerator of the system";
      parameter Real[n,n,:] den = fill(1e-10,n,n,o) "Denominator of the system";
      parameter Real[n,n] delay = zeros(n,n) "Delay of the system for every transfer function";
        Modelica.Blocks.Interfaces.RealInput u[n]
                                                 "Systems set point" annotation (
        Placement(transformation(extent={{-127,-18},{-92,18}}), iconTransformation(
              extent={{-120,-4},{-80,36}})));
           Modelica.Blocks.Interfaces.RealOutput y[n] "Systems real output" annotation (
        Placement(transformation(extent={{92,-18},{128,18}}), iconTransformation(
              extent={{-120,40},{-80,80}})));
      Transferfunctions.mimo_transferfunction system(
        n=n,
        num=num,
        den=den,
        delay = delay)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      connect(u,system.u);
      connect(system.y,y);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end mimo_processmodel;
  end Models;
  annotation (
    uses(Modelica(version = "3.2.2")));
end Masterthesis;
