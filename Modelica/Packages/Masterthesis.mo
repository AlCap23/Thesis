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
    model PIController
      "PI-Controller with optional inputs for gain scheduling"

      /******************** Connectors *****************************/

      Modelica.Blocks.Interfaces.RealInput u_s "Connector of setpoint input signal"
        annotation (Placement(transformation(extent={{-74,-30},{-54,-10}}, rotation=
               0), iconTransformation(extent={{-66,-10},{-46,10}})));
      Modelica.Blocks.Interfaces.RealInput u_m
        "Connector of measurement input signal" annotation (Placement(
            transformation(
            origin={0,-64},
            extent={{10,-10},{-10,10}},
            rotation=270), iconTransformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={-26,-58})));
      Modelica.Blocks.Interfaces.RealOutput y(start=yInitial)
        "Connector of actuator output signal" annotation (Placement(transformation(
              extent={{54,-10},{74,10}}, rotation=0), iconTransformation(extent={{54,
                -10},{74,10}})));
      Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput
        "true, if controller is on" annotation (Placement(transformation(extent={{-74,
                -50},{-54,-30}}, rotation=0), iconTransformation(extent={{-66,-48},{
                -46,-28}})));

      /*************************************************/

      Real u "Differenz between setpoint and measurement aka control error";

    public
      parameter String controllerType="P" "Controller Type" annotation (choices(
          choice="P" "P controller",
          choice="PI" "PI controller",
          choice="I" "I controller"), dialog(group="Settings"));

      parameter Boolean invertFeedback=false
        "true, if feedback Modelica.SIUnitsgnal is inverted"
        annotation (dialog(group="Settings"));

      parameter Real offset=0.0 "Operating point, added to proportional output"
        annotation (dialog(enable=(controllerType == "P"), group="Settings"));

      parameter String integralParameterType="Ti"
        "Controller Structure [k*(u + 1/Ti*integral(u))] or [k*u + ki*integral(u)]"
        annotation (choices(choice="Ti", choice="ki"), dialog(enable=((
              controllerType == "PI") or (controllerType == "I")), group="Settings"));

      parameter Real k=1 "Proportional gain of controller" annotation (Dialog(
            enable=(not use_kInput) and (not (controllerType == "I" and
              integralParameterType == "ki")), group="Setting parameters"));

      parameter Modelica.SIunits.Time Ti(min=Modelica.Constants.small) = 0.5
        "Time constant of Integrator block" annotation (Dialog(enable=(not
              use_TiInput and controllerType <> "P" and integralParameterType == "Ti"),
            group="Setting parameters"));

      parameter Modelica.SIunits.DampingCoefficient ki(min=Modelica.Constants.small)=
           0.5 "Integral gain" annotation (Dialog(enable=(not use_kiInput and
              controllerType <> "P" and integralParameterType == "ki"), group="Setting parameters"));

      parameter Boolean use_kInput=false "= true, if k defined by input"
        annotation (Dialog(
          enable=not (controllerType == "I" and integralParameterType == "ki"),
          tab="Advanced",
          group="Enable gain scheduling"));
      parameter Boolean use_TiInput=false "= true, if Ti defined by input"
        annotation (Dialog(
          enable=(controllerType <> "P" and integralParameterType == "Ti"),
          tab="Advanced",
          group="Enable gain scheduling"));
      parameter Boolean use_kiInput=false "= true, if ki defined by input"
        annotation (Dialog(
          enable=(controllerType <> "P" and integralParameterType == "ki"),
          tab="Advanced",
          group="Enable gain scheduling"));
      parameter Boolean use_setpointWeighting=false "Available for PI" annotation (
          Dialog(
          enable=(controllerType == "PI"),
          tab="Advanced",
          group="Setpoint weighting"));
      parameter Boolean use_bInput=false "True, if b is defined by external input"
        annotation (Dialog(enable = (use_setpointWeighting),tab="Advanced", group="Enable gain scheduling"));

      parameter Real b=0.5 "Setpoint weight 0 >= b >= 1" annotation (Dialog(
          enable=(use_setpointWeighting),
          tab="Advanced",
          group="Setpoint weighting"));

      /***************** Initialization *************************/

      parameter String initType="initialOutput" "Type of initialization"
        annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
          dialog(enable=controllerType <> "P", group="Initialization"));

      parameter Real yInitial=0 "Initial output of controller"
        annotation (dialog(enable=controllerType <> "P", group="Initialization"));

      /******************** Activation *****************************/
    public
      parameter Boolean use_activeInput=false
        "= true, if controller is switched on/off externally"
        annotation (dialog(group="Activation"));

      parameter Boolean use_y_notActive=false
        "= true, if output of not activated controller is defined externally. Otherwise output is hold at deactivation."
        annotation (dialog(group="Activation"));

      parameter Modelica.SIunits.Time activationTime=0.0
        "Time when controller is switched on"
        annotation (dialog(enable=not use_activeInput, group="Activation"));

    protected
      Modelica.Blocks.Routing.BooleanPassThrough getActive;
      Modelica.Blocks.Sources.BooleanExpression active_(y=time >= activationTime) if
                               not use_activeInput;

      /**********************************************************/

    protected
      Real y_unlim;
      Real y_old;
      Real u_antiWindUp;
      Real u_setpointWeighting;
      Real integral;
      Real ki_internal=if integralParameterType == "Ti" then k_int/
          ti_int else ki_int;

      //   PI_SP.Internals.GetInputs getInputs
      //     annotation (Placement(transformation(extent={{-20,16},{0,36}})));
      // Internal Connector
    protected
      Modelica.Blocks.Interfaces.RealInput k_int "Internal Use of kp";
      Modelica.Blocks.Interfaces.RealInput ki_int "Internal use of ki";
      Modelica.Blocks.Interfaces.RealInput b_int "Internal use of b";
      Modelica.Blocks.Interfaces.RealInput y_int "Internal use of y_notactive";
      Modelica.Blocks.Interfaces.RealInput ti_int "Internal use of ti";
      //Modelica.Blocks.Interfaces.BooleanInput act_int "Internal use of activation";
      // Sources for Parameter
    public
      Modelica.Blocks.Sources.Constant k_in_(k=k) if not use_kInput;
      Modelica.Blocks.Sources.Constant ki_in_(k=ki) if not use_kiInput;
      Modelica.Blocks.Sources.Constant Ti_in_(k=Ti) if not use_TiInput;
      Modelica.Blocks.Sources.Constant b_in_(k=b) if  not use_bInput;
      Modelica.Blocks.Sources.RealExpression y_notActive_(y=y_old) if not use_y_notActive;

    public
      Modelica.Blocks.Interfaces.RealInput k_in if use_kInput
        "External propotional gain" annotation (Placement(transformation(
              extent={{-74,30},{-54,50}}, rotation=0), iconTransformation(extent={{-66,
                30},{-46,50}})));
      Modelica.Blocks.Interfaces.RealInput ki_in if use_kiInput
        "External integral gain" annotation (Placement(transformation(
              extent={{-74,10},{-54,30}}, rotation=0), iconTransformation(extent={{-66,
                10},{-46,30}})));
      Modelica.Blocks.Interfaces.RealInput Ti_in if use_TiInput
        "External integral time constant" annotation (Placement(transformation(
              extent={{-74,10},{-54,30}}, rotation=0), iconTransformation(extent={{-74,
                10},{-54,30}})));
      Modelica.Blocks.Interfaces.RealInput b_in if use_bInput
        "External setpoint weight" annotation (Placement(transformation(
              extent={{-74,-10},{-54,10}}, rotation=0), iconTransformation(extent={{
                -66,-10},{-46,10}})));
      Modelica.Blocks.Interfaces.RealInput y_notActive if use_y_notActive
        "Controller output if not active" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,62}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,58})));
      Modelica.Blocks.Interfaces.RealInput u_a
        "Connector of anti-windup input signal" annotation (Placement(
            transformation(
            origin={40,-66},
            extent={{10,-10},{-10,10}},
            rotation=270), iconTransformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={26,-58})));

    initial equation

      if initType == "zeroIntegralState" then
        integral = 0;
      else
        if controllerType == "P" then
          integral = 0.0;
        elseif controllerType == "PI" then
          if use_setpointWeighting then
            integral = yInitial - k_int*u_setpointWeighting;
          else
            integral = yInitial - k_int*u;
          end if;
        else
          integral = yInitial;
        end if;
      end if;

      y_old = yInitial;

    equation


      if invertFeedback then
        u = (u_m - u_s);
        u_setpointWeighting = (u_m - b_int*u_s);
      else
        u = (u_s - u_m);
        u_setpointWeighting = (b_int*u_s - u_m);
      end if;

      if controllerType == "P" then
        der(integral) = 0;
        y_unlim = k_int*u + offset;
      elseif controllerType == "I" then
        der(integral) = if getActive.y then ki_internal*(u + u_antiWindUp) else 0.0;
        y_unlim = integral;
      else
        der(integral) = if getActive.y then ki_internal*(u + u_antiWindUp) else 0.0;
        if use_setpointWeighting then
          y_unlim = k_int*u_setpointWeighting + integral;
        else
          y_unlim = k_int*u + integral;
        end if;
      end if;

      //u_antiWindUp = if controllerType == "PI" then (y - y_unlim)/k_int else (y - y_unlim);
      u_antiWindUp = if abs(k_int) > 1e-10 then (if controllerType == "PI" then u_a/k_int else u_a) else u_a;

      //__________ Activation _________________________

      when not getActive.y and not initial() then
        y_old = pre(y);
      end when;

      when getActive.y then
        if controllerType == "PI" then
          reinit(integral, pre(y) - k_int*u);
        elseif controllerType == "I" then
          reinit(integral, pre(y));
        end if;
      end when;

      if getActive.y then
        y = smooth(0, y_unlim);
      else
        y = y_int;
      end if;

      //__________ Connections _________________________

      connect(activeInput, getActive.u);
      connect(active_.y, getActive.u);
      // Connect the Constants with the internals
      connect(k_in_.y, k_int);
      connect(ki_in_.y, ki_int);
      connect(Ti_in_.y, ti_int);
      connect(b_in_.y, b_int);
      connect(y_notActive_.y, y_int);
      // Connect the inputs for external definition with the internals
      connect(b_in, b_int);
      connect(k_in, k_int);
      connect(ki_in, ki_int);
      connect(Ti_in, ti_int);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-60,-60},{60,60}}),
            graphics={Bitmap(
              extent={{-60,-60},{60,60}},
              imageSource="iVBORw0KGgoAAAANSUhEUgAAAHgAAAB4CAIAAAC2BqGFAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAyklEQVR42u3SsQ3AIAxEUZwhvP98XsIpoUBUKErxfnft040hSfprsY6qInKxzNxDdzedmy+Oyfvg+CbQoEELNGjQCECDFmjQoAUatECDBi3QoAUaNGiBBi3QoEELNGiBBg1aoEELNGjQAg1aoEGDFmjQAg0atECDFmjQoAUatECDBi3QoAUaNGiBBi3QoEELNGiBBg1aoEELNGjQAg1aoEGDFmjQAg0atECDFmjQoAUatECDBi3QoAUaNGiBBi3QoEELNGiBliTp2AvV3wbfxPDYfgAAAABJRU5ErkJggg==",
              fileName="modelica://TIL/Images/PI_Controller_Icon.png"), Text(
              extent={{-60,30},{60,-30}},
              lineColor={0,0,0},
              textString="%controllerType")}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
        Documentation(info="<html>
<p>This controller can be used as P- or PI-controller. The controller can be switched on and off during the simulation. The output can be limited and an anti-wind-up function is included. The initialization can be set as steady state: der(y)=0 or with an initial output: y=yInitial. </p>
</html>"));

    end PIController;

    block miso_decentralcontroller
      "Creates an array of PI controller with multiple inputs and a single output"
      extends Modelica.Blocks.Icons.Block;
      /*************PARAMETER***************/
    public
      parameter Integer n(min=1) "Number of inputs"
        annotation (dialog(group="Settings"));
      outer parameter String controllerType="P" "Controller Type" annotation (choices(
          choice="P" "P controller",
          choice="PI" "PI controller",
          choice="I" "I controller"), dialog(group="Settings"));
      parameter Real KP[n]=ones(n) "Proportional Gain" annotation (dialog(enable=(
              not use_kInput) and (not (controllerType == "I")), group="Parameters"));
      parameter Real KI[n]=ones(n) "Integral Gain" annotation (dialog(enable=(not
              use_kiInput) and (not (controllerType == "P")), group="Parameters"));
      parameter Real B[n]=ones(n) "Set Point Weights" annotation (dialog(enable=(
              not use_bInput) or (use_setpointWeighting), group="Parameters"));
      parameter Real offset=0.0 "Operating point, added to proportional output"
        annotation (dialog(enable=(controllerType == "P"), group="Settings"));
      /**********************CONNECTOR*******************/
    public
      Modelica.Blocks.Interfaces.RealInput u_s[n] "Connector of Set Point value"
        annotation (Placement(transformation(extent={{-124,-46},{-84,-6}}),
            iconTransformation(extent={{-120,-38},{-88,-6}})));

      Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-8},{120,12}}),
            iconTransformation(extent={{80,-20},{120,20}})));

      Modelica.Blocks.Interfaces.RealInput u_m[n] "Connector of measurement value"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-58,-108}), iconTransformation(extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-58,-92})));

      Modelica.Blocks.Interfaces.RealInput u_a "Connector of anti-windup"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={60,-108}), iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={58,-92})));

      Modelica.Blocks.Interfaces.RealInput y_notActive if use_y_notActive
        "Activation" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,110}), iconTransformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));

      Modelica.Blocks.Interfaces.RealInput KP_in[n] if use_kInput "Activation"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-110,80}), iconTransformation(extent={{-118,72},{-90,100}})));

      Modelica.Blocks.Interfaces.RealInput KI_in[n] if use_kiInput "Activation"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-108,44}), iconTransformation(extent={{-148,24},{-90,82}})));

      Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput
        "Connector for active input" annotation (Placement(transformation(extent={{-124,
                -100},{-84,-60}}), iconTransformation(extent={{-122,-98},{-82,-58}})));

      Modelica.Blocks.Interfaces.RealInput B_in[n] if use_bInput
        "Connector of Set Point value" annotation (Placement(transformation(extent={
                {-128,-20},{-88,20}}), iconTransformation(extent={{-120,2},{-88,34}})));
      /*********************ACTIVATION********************/
    public
       parameter Boolean use_activeInput=false
        "True, if controller is switched on/off externally"
        annotation (Dialog(group="Activation"));
       parameter Boolean use_y_notActive=false
        "True, if output of not activated controller is defined externally"
        annotation (Dialog(group="Activation"));
      parameter Modelica.SIunits.Time activationTime=0.0
        "Time when controller is switched on"
        annotation (Dialog(group="Activation"));

       parameter Boolean use_kInput=false
        "True, if proportional gain is defined externally" annotation (Dialog(
          enable=not (controllerType == "I"),
          tab="Advanced",
          group="Gain Scheduling"));
       parameter Boolean use_kiInput=false
        "True, if integral gain is defined externally" annotation (Dialog(
          enable=not (controllerType == "P"),
          tab="Advanced",
          group="Gain Scheduling"));
       parameter Boolean use_bInput=false
        "True, if setpoint weight is defined externally"
        annotation (Dialog(tab="Advanced", group="Gain Scheduling"));
       parameter Boolean invertFeedback=false
        "true, if feedback Modelica.SIUnitsgnal is inverted"
        annotation (dialog(group="Settings"));
       parameter Boolean use_setpointWeighting=true "Available for PI"
        annotation (Dialog(
          enable=(controllerType == "PI") or (controllerType == "P"),
          tab="Advanced",
          group="Setpoint weighting"));
      /*********************INITIALIZATION****************/
       parameter String initType="initialOutput" "Type of initialization"
        annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
          dialog(enable=controllerType <> "P", group="Initialization"));
      parameter Real yInitial=0 "Initial output of controller"
        annotation (dialog(enable=controllerType <> "P", group="Initialization"));
      parameter Real[n] xInitial=zeros(n) "Initial states of controller"
        annotation (dialog(enable=(controllerType == "I") or (controllerType == "PI"),
            group="Initialization"));
      /*********************COMPONENTS********************/
    protected
      Modelica.Blocks.Math.MultiSum add(nu=n)
        annotation (Placement(transformation(extent={{24,-6},{36,6}})));
      PIController[n] pIController_sw(
        each controllerType=controllerType,
        each integralParameterType="ki",
        k=KP,
        ki=KI,
        each use_setpointWeighting=use_setpointWeighting,
        b=B,
        each initType=initType,
        each yInitial= yInitial,
        each use_activeInput=use_activeInput,
        each use_y_notActive=use_y_notActive,
        each activationTime=activationTime,
        each invertFeedback=invertFeedback,
        each use_kInput=use_kInput,
        each use_kiInput=use_kiInput,
        each use_bInput=use_bInput)
        annotation (Placement(transformation(extent={{-50,-6},{-38,6}})));
    //     Modelica.Blocks.Sources.RealExpression yna_in(y=0);
    // Modelica.Blocks.Sources.BooleanExpression active_in(y= false) if use_activeInput;
    equation
      // Connect Input Signals
      connect(u_s, pIController_sw.u_s);
      connect(u_m, pIController_sw.u_m);
      // Loop over all controller
      for inputs in 1:n loop
        // Connect each anti-windup to input
        connect(u_a, pIController_sw[inputs].u_a);
        // Connect activation
        if use_activeInput then
          //connect(active_in.y, activeInput);
          connect(activeInput, pIController_sw[inputs].activeInput);
    //       // Activation
    //       if use_y_notActive then
    //         connect(yna_in.y, y_notActive);
    //         connect(yna_in.y, pIController_sw[inputs].y_notActive);
    //       end if;
        end if;
      end for;
      // Connect exteral parameter conditionally
      if use_kInput then
        connect(KP_in, pIController_sw.k_in);
      end if;
      if use_kiInput then
        connect(KI_in, pIController_sw.ki_in);
      end if;
      if use_bInput then
        connect(B_in, pIController_sw.b_in);
      end if;
      // Connect output
      connect(pIController_sw.y, add.u);
      connect(add.y, y);

      annotation (Documentation(info="<html>
<p>
Block has a vector of continuous Real input signals and
one continuous Real output signal.
</p>
</html>"),     Icon(graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="k")}));
    end miso_decentralcontroller;

    block mimo_decentralcontroller
      extends Modelica.Blocks.Icons.Block;
      /**************PARAMETER**********/
    public
      parameter Integer n(min=1) "Number of inputs (= number of outputs)";
      inner parameter String controllerType="P" "Controller Type" annotation (
          choices(
          choice="P" "P controller",
          choice="PI" "PI controller",
          choice="I" "I controller"), dialog(group="Settings"));
      parameter Real[n,n] KP=identity(n) "Proportional Gain";
      parameter Real[n,n] KI=identity(n) "Integral Gain";
      parameter Real[n,n] B=identity(n) "Set Point Weights";
      parameter Real[n] offset=zeros(n)
        "Operating point, added to proportional output";

      /*************CONNECTOR************/
      // All
      Modelica.Blocks.Interfaces.RealInput u_s[n] "Connector of Set Point value"
        annotation (Placement(transformation(extent={{-122,-60},{-82,-20}}),
            iconTransformation(extent={{-122,-60},{-82,-20}})));
      Modelica.Blocks.Interfaces.RealInput u_m[n] "Connector of measurement value"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-60,-108}), iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-60,-96})));
      Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals"
        annotation (Placement(transformation(extent={{80,-20},{120,20}}),
            iconTransformation(extent={{80,-20},{120,20}})));
      // External
      Modelica.Blocks.Interfaces.RealInput B_in[n,n] if use_bInput "External defined B"
        annotation (Placement(transformation(extent={{-122,-20},{-82,20}}),
            iconTransformation(extent={{-122,-20},{-82,20}})));
      Modelica.Blocks.Interfaces.RealInput KI_in[n,n] if
                                                        use_kiInput "External defined KI."
        annotation (Placement(transformation(extent={{-130,20},{-90,60}}),
            iconTransformation(extent={{-122,20},{-82,60}})));
      Modelica.Blocks.Interfaces.RealInput KP_in[n,n] if use_kInput
                                                                   "External defined KP."
        annotation (Placement(transformation(extent={{-130,60},{-90,100}}),
            iconTransformation(extent={{-122,60},{-82,100}})));
      Modelica.Blocks.Interfaces.RealInput activeInput if use_activeInput "activeInput" annotation (
          Placement(transformation(extent={{-122,-100},{-82,-60}}),
            iconTransformation(extent={{-122,-100},{-82,-60}})));


      /*********************ACTIVATION********************/
    public
      inner parameter Boolean use_activeInput=false
        "True, if controller is switched on/off externally"
        annotation (Dialog(group="Activation"));
      inner parameter Boolean use_y_notActive=false
        "True, if output of not activated controller is defined externally"
        annotation (Dialog(group="Activation"));
      parameter Modelica.SIunits.Time activationTime=0.0
        "Time when controller is switched on"
        annotation (Dialog(group="Activation"));

      inner parameter Boolean use_kInput=false
        "True, if proportional gain is defined externally" annotation (Dialog(
          enable=not (controllerType == "I"),
          tab="Advanced",
          group="Gain Scheduling"));
      inner parameter Boolean use_kiInput=false
        "True, if integral gain is defined externally" annotation (Dialog(
          enable=not (controllerType == "P"),
          tab="Advanced",
          group="Gain Scheduling"));
      inner parameter Boolean use_bInput=false
        "True, if setpoint weight is defined externally"
        annotation (Dialog(tab="Advanced", group="Gain Scheduling"));
      inner parameter Boolean invertFeedback=false
        "true, if feedback Modelica.SIUnitsgnal is inverted"
        annotation (dialog(group="Settings"));
      inner parameter Boolean use_setpointWeighting=true "Available for PI"
        annotation (Dialog(
          enable=(controllerType == "PI") or (controllerType == "P"),
          tab="Advanced",
          group="Setpoint weighting"));
      /*********************INITIALIZATION****************/
      inner parameter String initType="initialOutput" "Type of initialization"
        annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
          dialog(enable=controllerType <> "P", group="Initialization"));
      parameter Real yInitial=0 "Initial output of controller"
        annotation (dialog(enable=controllerType <> "P", group="Initialization"));
      parameter Real[n] xInitial=zeros(n) "Initial states of controller"
        annotation (dialog(enable=(controllerType == "I") or (controllerType == "PI"),
            group="Initialization"));
      /*********************COMPONENTS********************/

      Controller.miso_decentralcontroller[n] pi(
        each n=n,
        KP=KP,
        KI=KI,
        B=B) annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
      Modelica.Blocks.Interfaces.RealInput u_a[n] "Connector of anti-windup inputs"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={60,-106}), iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={58,-94})));

    equation
      for outputs in 1:n loop
        connect(u_s, pi[outputs].u_s);
        connect(u_m, pi[outputs].u_m);
        connect(pi[outputs].y, y[outputs]);
        connect(u_a[outputs],pi[outputs].u_a);
      end for;

      annotation (
        Documentation(info="<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="K")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end mimo_decentralcontroller;

    block mimo_decentralcontroller_2 "MIMO PI Controller with optional inputs for gain scheduling"
      extends Modelica.Blocks.Icons.Block;
      /*****************SETTING*****************/
    public
      parameter Integer n(min = 1) "Number of in- and outputs";
      parameter String controllerType="P" "Controller Type" annotation (choices(
          choice="P" "P controller",
          choice="PI" "PI controller",
          choice="I" "I controller"), dialog(group="Settings"));
      parameter Boolean use_KP_in=false " True for external proportional gain";
      parameter Boolean use_KI_in=false "True for external integral gain";
      parameter Boolean use_B_in = false "True for external setpoint-weight";
      parameter Boolean use_Activation = false "True for external activation";
      parameter Boolean invert_Feedback = false "True, if feedback is negative";
      parameter Boolean use_y_Inactive = false "True, if output of inactive controller is defined externally";
      /*****************VARIABLES****************/
      parameter Real[n] offset=zeros(n)
        "Operating point, added to proportional output"
        annotation (dialog(enable=(controllerType == "P"), group="Settings"));
      parameter Real[n] y_start=zeros(n) "Initial value of controller output"
        annotation (Dialog(enable=init == Modelica.Blocks.Init.SteadyState or
              initType == Modelica.Blocks.Init.InitialOutput, group=
              "Initialization"));
      parameter Real[n] x_start=zeros(n) "Initial value of controller states";
      parameter Real[n,n] KP = identitiy(n) if not use_KP_in;
      parameter Real[n,n] KI = identitiy(n) if not use_KI_in;
      parameter Real[n,n] B = identity(n) if not use_B_in;
      /*****************CONNECTORS*******************/
      Modelica.Blocks.Interfaces.RealInput[n,n] KP_in if use_KP_in annotation (
          Placement(transformation(extent={{-128,70},{-88,110}}),
            iconTransformation(extent={{-128,70},{-88,110}})));
      Modelica.Blocks.Interfaces.RealInput[n,n] KI_in if use_KI_in annotation (
          Placement(transformation(extent={{-128,42},{-88,82}}), iconTransformation(
              extent={{-128,42},{-88,82}})));
      Modelica.Blocks.Interfaces.RealInput[n,n] B_in if use_B_in annotation (
          Placement(transformation(extent={{-130,14},{-90,54}}), iconTransformation(
              extent={{-130,14},{-90,54}})));
      Modelica.Blocks.Interfaces.BooleanInput[n] active_Input if use_Activation
        " True, if controller is on" annotation (Placement(transformation(extent={{
                -74,-50},{-54,-30}}, rotation=0), iconTransformation(extent={{-66,-48},
                {-46,-28}})));
      Modelica.Blocks.Interfaces.RealInput[n] u_s "Setpoint Input" annotation (
          Placement(transformation(extent={{-130,-20},{-90,20}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-110,0})));
      Modelica.Blocks.Interfaces.RealInput[n] u_m "Measurement Input" annotation (
          Placement(transformation(extent={{-130,-60},{-90,-20}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-110,-40})));
      Modelica.Blocks.Interfaces.RealInput[n] u_a "Anti-Windup Input" annotation (
          Placement(transformation(extent={{-128,-100},{-88,-60}}),
            iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-108,-80})));
      Modelica.Blocks.Interfaces.RealOutput[n] y(start=y_start) "Controller Output" annotation (Placement(
            transformation(extent={{94,-20},{134,20}}), iconTransformation(extent={{
                94,-20},{134,20}})));
      // Internal Variables for KP, KI, B
    protected
      Modelica.Blocks.Interfaces.RealInput[n,n] KP_internal;
      Modelica.Blocks.Interfaces.RealInput[n,n] KI_internal;
      Modelica.Blocks.Interfaces.RealInput[n,n] B_internal;
      Modelica.Blocks.Interfaces.BooleanInput[n] y_internal;
      // Internal States
      Real[n] x(start=x_start) "Integral Part of the Output";



    equation
      // Connections for the variables
      connect(KP_in, KP_internal);
      if not use_KP_in then
        KP_internal = KP;
      end if;
      connect(KI_in, KI_internal);
      if not use_KI_in then
        KI_internal = KI;
      end if;
      connect(B_in, B_internal);
      if not use_B_in then
        B_internal = B;
      end if;

    end mimo_decentralcontroller_2;

    model mimo_decoupler "Decoupler for MIMO Systems"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer n(min=1) "Number of inputs";
      parameter Real[n,n] D "Decoupling Matrix";
      parameter Real[n] ymax "Maximum Output";
      parameter Real[n] ymin "Minimum Output";
      Modelica.Blocks.Interfaces.RealInput u[n] "Connector of Real input signals" annotation (Placement(
            transformation(extent={{-120,-20},{-80,20}}), iconTransformation(extent=
               {{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{80,-20},{120,20}}), iconTransformation(extent={{
                80,-20},{120,20}})));

      Modelica.Blocks.Interfaces.RealOutput y_a[n]
        "Connector of anti-windup output" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,-100}), iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,-100})));
      Modelica.Blocks.Nonlinear.Limiter limiter[n](uMax=ymax, uMin=ymin)
        annotation (Placement(transformation(extent={{2,-10},{22,10}})));


    equation

    limiter.y = y; // Output of the limiter is system output
    limiter.u = D*u; // Input of the limiter is D*u
    y_a = (Modelica.Math.Matrices.inv(D)*(limiter.y)- u);// Output for Controller


      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
              extent={{-50,60},{42,-54}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="D")}),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end mimo_decoupler;

    block mimo_decentralcontroller_2_old
      "MIMO PI Controller with optional inputs for gain scheduling"
      extends Modelica.Blocks.Icons.Block;

      /*******************CONNECTORS*********************/
      Modelica.Blocks.Interfaces.RealInput u_s[n] "Connector of Set Point value"
        annotation (Placement(transformation(extent={{-120,40},{-80,80}}),
            iconTransformation(extent={{-120,40},{-80,80}})));

      Modelica.Blocks.Interfaces.RealInput u_m[n] "Connector of measurement value"
        annotation (Placement(transformation(extent={{-120,-80},{-80,-40}}),
            iconTransformation(extent={{-120,-80},{-80,-40}})));
      Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals"
        annotation (Placement(transformation(extent={{80,-20},{120,20}}),
            iconTransformation(extent={{80,-20},{120,20}})));
      Modelica.Blocks.Interfaces.RealInput u_a[n] "Connector of anti-windup inputs"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={0,-100}), iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={0,-96})));
      Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput
        "true, if controller is on" annotation (Placement(transformation(extent={{-12,
                90},{8,110}}, rotation=0), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={50,96})));
      Modelica.Blocks.Interfaces.RealInput kInput[n,n] "True, if external input"
      annotation (Placement(transformation(extent={{-12,62},{8,82}},
                              rotation=0), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,96})));
      Modelica.Blocks.Interfaces.RealInput kiInput[n,n] "True, if external input"
      annotation (Placement(transformation(extent={{-12,62},{8,82}},
                              rotation=0), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,96})));
      /**************************************************/
      /*************SETTINGS***********/
    public
      parameter String controllerType="P" "Controller Type" annotation (choices(
            choice="P" "P controller", choice="PI" "PI controller",  choice="I"
            "I controller"),                                                                     dialog(group=
              "Settings"));
      parameter Boolean invertFeedback=false "True, if feedback signal is inverted"
        annotation (dialog(group="Settings"));
      parameter Real[n] offset=zeros(n)
        "Operating point, added to proportional output"
        annotation (dialog(enable=(controllerType == "P"), group="Settings"));
      parameter Integer n(min=1) "Number of in- and outputs of the controller";
      parameter Boolean use_kInput=false
        "True, if proportional gain is defined by Input" annotation (Dialog(
          enable=not (controllerType == "I" and integralParameterType == "ki"),
          tab="Advanced",
          group="Enable gain scheduling"));
      parameter Boolean use_kiInput=false
        "True, if integral gain is defined by Input" annotation (Dialog(
          enable=not (controllerType == "I" and integralParameterType == "ki"),
          tab="Advanced",
          group="Enable gain scheduling"));
      parameter Boolean use_SetPointWeight=true
        "True, if setpoint-weight is enabled" annotation (Dialog(
          enable=(controllerType == "PI"),
          tab="Advanced",
          group="Setpoint weighting"));
      /******************************/
      /***********INITIALIZATION*****/
      parameter Modelica.Blocks.Types.Init init=Modelica.Blocks.Types.Init.NoInit
        "Initialization of controller";
      parameter Real[n] y_start=zeros(n) "Initial value of controller output"
        annotation (Dialog(enable=init == Modelica.Blocks.Init.SteadyState or
              initType == Modelica.Blocks.Init.InitialOutput, group="Initialization"));
      parameter Real[n] x_start=zeros(n) "Initial value of controller states";
      /******************************/
      /************PARAMETER*********/
      parameter Real KP[n,n]=identity(n) "Proportional Gain" annotation(Dialog(enable = not use_kInput));
      parameter Real KI[n,n]=identity(n)
        "Integral Gain" annotation(Dialog(enable = not use_kiInput));
      parameter Real B[n,n]=identity(n) "Set Point Weights";
      /*****************************/
      /************ACTIVATION*******/
    public
      parameter Boolean use_activeInput=false
        "= true, if controller is switched on/off externally"
        annotation (dialog(group="Activation"));
      parameter Boolean use_y_notActive=false
        "= true, if output of not activated controller is defined externally. Otherwise output is hold at deactivation."
        annotation (dialog(group="Activation"));
      parameter Modelica.SIunits.Time activationTime=0.0
        "Time when controller is switched on"
        annotation (dialog(enable=not use_activeInput, group="Activation"));
    protected
      Modelica.Blocks.Routing.BooleanPassThrough getActive;
      Modelica.Blocks.Sources.BooleanExpression active_(y=time >= activationTime) if
                               not use_activeInput;
      /****************************/

    protected
      Real[n] x(start=x_start) "Integral Part of the Output";
      Real[n] y_old "Previous value of y";
      Real[n] x_old "Previous value of x";
      Real[n,n] KPin "Internal KP";
      Real[n,n] KIin "Internal KI";
      //Modelica.Blocks.Sources.RealExpression y_notActive(y=y_old) if not use_y_notActive;
    initial equation
      if init == Modelica.Blocks.Types.Init.SteadyState then
        der(x) = zeros(n);
      elseif init == Modelica.Blocks.Types.Init.InitialState then
        x = x_start;
      elseif init == Modelica.Blocks.Types.Init.InitialOutput then
        y = y_start;
      end if;
      y_old = y_start;
      x_old = x_start;
    equation
      /*************CONNECTIONS*********************/
      // Activation
      connect(activeInput, getActive.u);
      connect(active_.y, getActive.u);
      // Parameter
      if use_kInput then
        KPin = kInput;
      else
        KPin =KP;
      end if;
      if use_kiInput then
        KIin = kiInput;
      else
        KIin =KI;
      end if;

      if invertFeedback then
        if controllerType == "P" then
          der(x) = zeros(n);
          y = KPin*(u_m-B*u_s)+ x + offset;
        elseif controllerType =="I" then
          der(x) = KIin*(u_m-B*u_s) + KPin*u_a;
          y = x + offset;
        else
          der(x) = KIin*(u_m-B*u_s) + KPin*u_a;
          y = KPin*(u_m-B*u_s)+ x + offset;
        end if;
      else
        if controllerType == "P" then
          der(x) = zeros(n);
          y = KPin*(-u_m+B*u_s)+ x + offset;
        elseif controllerType =="I" then
          der(x) = KIin*(-u_m+B*u_s) + KPin*u_a;
          y = x + offset;
        else
          der(x) = KIin*(-u_m+B*u_s) + KPin*u_a;
          y = KPin*(-u_m+B*u_s)+ x + offset;
        end if;
      end if;
    //   /**Operational Mode**/
    //   if getActive.y then
    //   /************INVERTED FEEDBACK***************/
    //   if invertFeedback then
    //     // P - Controller
    //     if controllerType == "P" then
    //       // Pure Proportional Gain
    //       der(x) = zeros(n);
    //       y = KPin*(u_m - B*u_s) + x + offset;
    //
    //     elseif controllerType == "I" then
    //       // Intregral Part is given
    //       der(x) = KIin*(u_m - u_s) + KPin*u_a;
    //       // Pure Integral Output
    //       y = x + offset;
    //     else
    //       // Intregral Part is given
    //       der(x) = KIin*(u_m - u_s) + KPin*u_a;
    //       // Combination of Integral and Proportional
    //       y = KPin*(u_m - B*u_s) + x + offset;
    //       // Output
    //     end if;
    //     /**************NORMAL FEEDBACK*************/
    //   else
    //     // P - Controller
    //     if controllerType == "P" then
    //       // Pure Proportional Gain
    //       der(x) = zeros(n);
    //       y = KPin*(-u_m + B*u_s) + x + offset;
    //
    //     elseif controllerType == "I" then
    //       // Intregral Part is given
    //       der(x) = KIin*(-u_m + u_s) + KPin*u_a;
    //       // Pure Integral Output
    //       y = x + offset;
    //     else
    //       // Intregral Part is given
    //       der(x) = KIin*(u_m - u_s) + KPin*u_a;
    //       // Combination of Integral and Proportional
    //       y = KPin*(-u_m + B*u_s) + x + offset;
    //       // Output
    //     end if;
    //   end if;
    //   else
    //     y = 0.0;
    //   end if;
      /*************ACTIVATION**************/
    //    when not getActive.y and not initial() then
    //      y_old = pre(y);
    //      x_old = pre(x);
    //    end when;
    //
    //    // When controller is activated
    //    when getActive.y then
    //      // PI Controller
    //      if controllerType == "PI" then
    //        if invertFeedback then
    //          // Initialize with former gains
    //          reinit(x, pre(y) - KPin*(-B*u_s + u_m));
    //        else
    //          reinit(x, pre(y) - KPin*(B*u_s - u_m));
    //        end if;
    //      elseif controllerType == "I" then
    //        reinit(x, pre(y));
    //      end if;
    //    end when;
      /*************************************/
      annotation (
        Documentation(info="<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
              extent={{-52,64},{40,-50}},
              lineColor={0,0,0},
              textStyle={TextStyle.Bold},
              textString="K")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end mimo_decentralcontroller_2_old;
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

    model Test_SISOCN "Tester for SISO system"
      Controller.PIController pIController(
        controllerType="PI",
        invertFeedback=false,
        integralParameterType="ki",
        k=1,
        ki=0.1,
        initType="zeroIntegralState",
        use_setpointWeighting=true,
        use_activeInput=true,
        use_y_notActive=true)
        annotation (Placement(transformation(extent={{-10,8},{2,20}})));
      Modelica.Blocks.Sources.Step u_r(
        height=1,
        offset=0,
        startTime=2)
        annotation (Placement(transformation(extent={{-76,4},{-56,24}})));
      Modelica.Blocks.Sources.RealExpression u_m
        annotation (Placement(transformation(extent={{-84,-54},{-64,-34}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1)
        annotation (Placement(transformation(extent={{26,8},{46,28}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={38,-22})));
      Modelica.Blocks.Sources.RealExpression u_m1(y=0.75)
        annotation (Placement(transformation(extent={{-40,48},{-20,68}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=2)
        annotation (Placement(transformation(extent={{-76,28},{-56,48}})));
      Modelica.Blocks.Logical.Not not1
        annotation (Placement(transformation(extent={{-46,28},{-26,48}})));
    equation
      connect(u_m.y, pIController.u_m) annotation (Line(points={{-63,-44},{-63,
              -8},{-6.6,-8},{-6.6,8.2}}, color={0,0,127}));
      connect(pIController.y, limiter.u)
        annotation (Line(points={{2.4,14},{2.4,18},{24,18}}, color={0,0,127}));
      connect(add.u2, pIController.y) annotation (Line(points={{32,-10},{20,-10},
              {20,14},{2.4,14}}, color={0,0,127}));
      connect(add.u1, limiter.y)
        annotation (Line(points={{44,-10},{47,-10},{47,18}}, color={0,0,127}));
      connect(add.y, pIController.u_a) annotation (Line(points={{38,-33},{38,
              -33},{38,-60},{38,-62},{-1.4,-62},{-1.4,8.2}}, color={0,0,127}));
      connect(pIController.u_s, u_r.y) annotation (Line(points={{-9.6,14},{-32,
              14},{-55,14}}, color={0,0,127}));
      connect(pIController.y_notActive, u_m1.y) annotation (Line(points={{-4,
              19.8},{-4,19.8},{-4,58},{-19,58}}, color={0,0,127}));
      connect(booleanStep.y, not1.u) annotation (Line(points={{-55,38},{-52,38},
              {-48,38}}, color={255,0,255}));
      connect(pIController.activeInput, not1.y) annotation (Line(points={{-9.6,
              10.2},{-18,10.2},{-18,38},{-25,38}}, color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_SISOCN;

    model Test_MISOCN "Tester for MISO controller"

      Controller.miso_decentralcontroller miso_decentralcontroller1(
        n=2,
        controllerType="PI",
        invertFeedback=false,
        B={0,0},
        KP={1,2},
        KI={0.1,0.01})
        annotation (Placement(transformation(extent={{12,22},{32,42}})));
      Modelica.Blocks.Sources.Step step(
        startTime=150,
        height=1,
        offset=0)
        annotation (Placement(transformation(extent={{-74,64},{-54,84}})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-46,-20},{-26,0}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(        uMin=-1, uMax=1)
        annotation (Placement(transformation(extent={{50,22},{70,42}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={60,0})));
    equation
      connect(miso_decentralcontroller1.y, limiter.u) annotation (Line(points={{32,32},
              {42,32},{48,32}},                 color={0,0,127}));
      connect(add.u2, miso_decentralcontroller1.y) annotation (Line(points={{54,12},
              {38,12},{38,32},{32,32}},   color={0,0,127}));
      connect(add.u1, limiter.y) annotation (Line(points={{66,12},{86,12},{86,
              32},{71,32}},
                        color={0,0,127}));
      connect(add.y, miso_decentralcontroller1.u_a) annotation (Line(points={{
              60,-11},{60,-12},{60,-28},{27.8,-28},{27.8,22.8}}, color={0,0,127}));
      connect(miso_decentralcontroller1.u_m[1], const.y) annotation (Line(
            points={{16.2,21.8},{-4,21.8},{-4,-10},{-25,-10}}, color={0,0,127}));
      connect(miso_decentralcontroller1.u_m[2], const.y) annotation (Line(
            points={{16.2,23.8},{-4,23.8},{-4,-10},{-25,-10}}, color={0,0,127}));
      connect(miso_decentralcontroller1.u_s[1], step.y) annotation (Line(points
            ={{11.6,29},{-14,29},{-20,29},{-20,74},{-53,74}}, color={0,0,127}));
      connect(miso_decentralcontroller1.u_s[2], step.y) annotation (Line(points
            ={{11.6,30.6},{-20,30.6},{-20,74},{-53,74}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MISOCN;

    model Test_MIMOCN "Tester for MIMO Controller"

      Controller.mimo_decentralcontroller mimo_decentralcontroller1(
        n=2,
        KP={{1,0},{0,1}},
        controllerType="PI",
        KI={{0.1,0},{0,0.1}},
        B={{0,0},{0,0}})
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Blocks.Sources.Step step(height=5)
        annotation (Placement(transformation(extent={{-86,24},{-66,44}})));
      Modelica.Blocks.Sources.Step step1(height=10, startTime=5)
        annotation (Placement(transformation(extent={{-86,52},{-66,72}})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-84,-20},{-64,0}})));
    equation
      connect(const.y, mimo_decentralcontroller1.u_m[1]) annotation (Line(points={{-63,-10},
              {-42,-10},{-42,-10.6},{-8,-10.6}}, color={0,0,127}));
      connect(const.y, mimo_decentralcontroller1.u_m[2]) annotation (Line(points={{-63,-10},
              {-42,-10},{-42,-8.6},{-8,-8.6}},   color={0,0,127}));
      connect(step1.y, mimo_decentralcontroller1.u_s[2]) annotation (Line(points={{-65,62},
              {-40,62},{-40,-3},{-12.2,-3}}, color={0,0,127}));
      connect(step.y, mimo_decentralcontroller1.u_s[1]) annotation (Line(points={{-65,34},
              {-42,34},{-42,-5},{-12.2,-5}},   color={0,0,127}));
      connect(const.y, mimo_decentralcontroller1.u_a[1]) annotation (Line(
            points={{-63,-10},{-32,-10},{-32,-28},{3.8,-28},{3.8,-10.4}}, color
            ={0,0,127}));
      connect(const.y, mimo_decentralcontroller1.u_a[2]) annotation (Line(
            points={{-63,-10},{-42,-10},{-42,-66},{3.8,-66},{3.8,-8.4}}, color=
              {0,0,127}));
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

    model Test_CL

      parameter Integer n=2 "Number of in- and outputs";
      parameter Integer o=1 "Order of the Process";
      parameter Real[n,n,:] num = fill(1e-10,n,n,1) "Numerator of the system";
      parameter Real[n,n,:] den = fill(1e-10,n,n,o) "Denominator of the system";
      parameter Real[n,n] delay = zeros(n,n) "Delay of the system for every transfer function";
      parameter Real[n,n] kp = fill(1e-10,n,n) "Proportional gain of the controller";
      parameter Real[n,n] ki = fill(1e-10,n,n) "Integral gain of the controller";
      parameter Real[n,n] b =  fill(1e-10,n,n) "Set Point Weight of the controller";
      parameter Real[n,n] d =  identity(n) "Decoupler of the System";
      parameter Real[n] ymax = fill(1e10,n) "Maximum Output of the System";
      parameter Real[n] ymin = fill(1e10,n) "Minimum Output of the System";

      Transferfunctions.mimo_transferfunction System(
        n=2,
        delay=fill(
                1e-10,
                2,
                2),
        den={{{10,1},{20,5}},{{7,1},{10,3}}},
        num={{{1},{1e-10}},{{1e-10},{2}}})
                       annotation (Placement(transformation(extent={{40,0},{60,
                20}})));
      Controller.mimo_decentralcontroller Decentral_Controller(
        n=2,
        KP=[0.725,1e-10; 1e-10,0.725],
        KI=[0.852,1e-10; 1e-10,0.852],
        B=[0,1e-10; 1e-10,0])
               annotation (Placement(transformation(extent={{-36,0},{-16,20}})));

      Controller.mimo_decoupler Decoupler(
        n=2,
        ymin={-5,-5},
        ymax={5,3},
        D=[1,0; 0,1])
        annotation (Placement(transformation(extent={{2,0},{22,20}})));
      Modelica.Blocks.Sources.Step step(
        startTime=1,
        offset=0,
        height=2)
        annotation (Placement(transformation(extent={{-102,-30},{-82,-10}})));
      Modelica.Blocks.Sources.Step step1(
        startTime=5,
        offset=0,
        height=2)
        annotation (Placement(transformation(extent={{-100,8},{-80,28}})));
    equation
      for inputs in 1:n loop
      end for;
      connect(Decentral_Controller.y, Decoupler.u)
        annotation (Line(points={{-16,10},{-7,10},{2,10}}, color={0,0,127}));
      connect(Decoupler.y, System.u)
        annotation (Line(points={{22,10},{40,10}},         color={0,0,127}));
      connect(System.y, Decentral_Controller.u_m) annotation (Line(points={{60,10},
              {60,10},{80,10},{80,-20},{-50,-20},{-50,4},{-36,4}},color={0,0,127}));
      connect(Decoupler.y_a, Decentral_Controller.u_a) annotation (Line(points={{12,
              0},{12,-10},{-26,-10},{-26,0.4}}, color={0,0,127}));
      connect(Decentral_Controller.u_s[1], step.y) annotation (Line(points={{
              -36,15},{-58,15},{-58,-20},{-81,-20}}, color={0,0,127}));
      connect(Decentral_Controller.u_s[2], step1.y) annotation (Line(points={{
              -36,17},{-56,17},{-56,18},{-79,18}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_CL;

    model Test_CL_2


      Controller.mimo_decentralcontroller_2 mimo_decentralcontroller_2_1(
          use_KP_in=false)
        annotation (Placement(transformation(extent={{-10,-4},{10,16}})));
      Modelica.Blocks.Sources.Constant const(k=5)
        annotation (Placement(transformation(extent={{-96,-20},{-76,2}})));
      Modelica.Blocks.Sources.Constant const1(k=2)
        annotation (Placement(transformation(extent={{-86,14},{-66,36}})));
    equation
      connect(mimo_decentralcontroller_2_1.u[1], const1.y) annotation (Line(
            points={{-10.8,6},{-28,6},{-28,50},{-65,50},{-65,25}}, color={0,0,
              127}));
      connect(mimo_decentralcontroller_2_1.u[2], const.y) annotation (Line(
            points={{-10.8,6},{-38,6},{-38,-36},{-75,-36},{-75,-9}}, color={0,0,
              127}));
    end Test_CL_2;
  end Testers;

  package Models "Package for used models"
    model mimo_closedloop

      parameter Integer n=2 "Number of in- and outputs";
      parameter Integer o=9 "Order of the Process";
      parameter Real[n,n,:] num = fill(1e-10,n,n,1) "Numerator of the system";
      parameter Real[n,n,:] den = fill(1e-10,n,n,o) "Denominator of the system";
      parameter Real[n,n] delay = zeros(n,n) "Delay of the system for every transfer function";
      parameter Real[n,n] kp = fill(1e-10,n,n) "Proportional gain of the controller";
      parameter Real[n,n] ki = fill(1e-10,n,n) "Integral gain of the controller";
      parameter Real[n,n] b =  fill(1e-10,n,n) "Set Point Weight of the controller";
      parameter Real[n,n] d =  fill(1e-10,n,n) "Decoupler of the System";
      parameter Real[n] ymax = fill(1e10,n) "Maximum Output of the System";
      parameter Real[n] ymin = fill(1e10,n) "Minimum Output of the System";

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

      Controller.mimo_decoupler Decoupler(n=n, D=d, ymax = ymax, ymin = ymin)
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
      connect(Decoupler.y_a, Decentral_Controller.u_a) annotation (Line(points={{12,
              0},{12,-10},{-26,-10},{-26,0.4}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end mimo_closedloop;

    model mimo_processmodel
      "Processmodel for a multiple input, multiple output system"

      parameter Integer n=2 "Number of in- and outputs";
      parameter Integer o=9 "Order of the Process";
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
