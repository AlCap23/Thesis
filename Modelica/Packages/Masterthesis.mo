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
          //reinit(integral, pre(y) - k_int*u);
          reinit(integral,pre(y));
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

    package MISO_Controller "Pacakage for MISO Controller"

      block miso_decentralcontroller_outer
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
         outer parameter Boolean use_activeInput=true
          "True, if controller is switched on/off externally"
          annotation (Dialog(group="Activation"));
         outer parameter Boolean use_y_notActive=false
          "NOT IMPLEMENTED"
          annotation (Dialog(group="Activation"));
        parameter Modelica.SIunits.Time activationTime=0.0
          "Time when controller is switched on"
          annotation (Dialog(group="Activation"));

         outer parameter Boolean use_kInput=false
          "True, if proportional gain is defined externally" annotation (Dialog(
            enable=not (controllerType == "I"),
            tab="Advanced",
            group="Gain Scheduling"));
         outer parameter Boolean use_kiInput=false
          "True, if integral gain is defined externally" annotation (Dialog(
            enable=not (controllerType == "P"),
            tab="Advanced",
            group="Gain Scheduling"));
         outer parameter Boolean use_bInput=false
          "True, if setpoint weight is defined externally"
          annotation (Dialog(tab="Advanced", group="Gain Scheduling"));
         outer parameter Boolean invertFeedback=false
          "true, if feedback Modelica.SIUnitsgnal is inverted"
          annotation (dialog(group="Settings"));
         outer parameter Boolean use_setpointWeighting=true "Available for PI"
          annotation (Dialog(
            enable=(controllerType == "PI") or (controllerType == "P"),
            tab="Advanced",
            group="Setpoint weighting"));
        /*********************INITIALIZATION****************/
         outer parameter String initType="initialOutput" "Type of initialization"
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
</html>"),       Icon(graphics={Text(
                extent={{-52,64},{40,-50}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="k")}));
      end miso_decentralcontroller_outer;

      block miso_decentralcontroller_single
        "Creates an array of PI controller with multiple inputs and a single output"
        extends Modelica.Blocks.Icons.Block;
        /*************PARAMETER***************/
      public
        parameter Integer n(min=1) "Number of inputs"
          annotation (dialog(group="Settings"));
        parameter String controllerType="P" "Controller Type" annotation (choices(
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
              origin={-58,-108}), iconTransformation(
              extent={{-20,-20},{20,20}},
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
              origin={0,110}), iconTransformation(
              extent={{-20,-20},{20,20}},
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
        parameter Boolean use_activeInput=true
          "True, if controller is switched on/off externally"
          annotation (Dialog(group="Activation"));
        parameter Boolean use_y_notActive=false
          "NOT IMPLEMENTED"
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
        parameter Boolean use_setpointWeighting=true "Available for PI" annotation (
            Dialog(
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
          each yInitial=yInitial,
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
</html>"),       Icon(graphics={Text(
                extent={{-52,64},{40,-50}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="k")}));
      end miso_decentralcontroller_single;
    end MISO_Controller;

    package MIMO_Controller

      block mimo_decentralcontroller_outer
        extends Modelica.Blocks.Icons.Block;
        /**************PARAMETER**********/
      public
        outer parameter Integer n(min=1) "Number of inputs (= number of outputs)";
        outer parameter String controllerType="P" "Controller Type" annotation (
            choices(
            choice="P" "P controller",
            choice="PI" "PI controller",
            choice="I" "I controller"), dialog(group="Settings"));
        outer parameter Real[n,n] KP=identity(n) "Proportional Gain";
        outer parameter Real[n,n] KI=identity(n) "Integral Gain";
        outer parameter Real[n,n] B=identity(n) "Set Point Weights";
        outer parameter Real[n] offset=zeros(n)
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
                Modelica.Blocks.Interfaces.RealInput u_a[n] "Connector of anti-windup inputs"
          annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={60,-106}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={58,-94})));
        Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals"
          annotation (Placement(transformation(extent={{80,-20},{120,20}}),
              iconTransformation(extent={{80,-20},{120,20}})));
        // External
        Modelica.Blocks.Interfaces.RealInput[n,n] B_in if use_bInput "External defined B"
          annotation (Placement(transformation(extent={{-122,-20},{-82,20}}),
              iconTransformation(extent={{-122,-20},{-82,20}})));
        Modelica.Blocks.Interfaces.RealInput[n,n] KI_in if
                                                          use_kiInput "External defined KI."
          annotation (Placement(transformation(extent={{-130,20},{-90,60}}),
              iconTransformation(extent={{-122,20},{-82,60}})));
        Modelica.Blocks.Interfaces.RealInput[n,n] KP_in if use_kInput
                                                                     "External defined KP."
          annotation (Placement(transformation(extent={{-130,60},{-90,100}}),
              iconTransformation(extent={{-122,60},{-82,100}})));
        Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput "activeInput" annotation (
            Placement(transformation(extent={{-122,-100},{-82,-60}}),
              iconTransformation(extent={{-122,-100},{-82,-60}})));

        /*********************ACTIVATION********************/
      public
        outer parameter Boolean use_activeInput=false
          "True, if controller is switched on/off externally"
          annotation (Dialog(group="Activation"));
        inner parameter Boolean use_y_notActive=false
          "NOT IMPLEMENTED"
          annotation (Dialog(group="Activation"));
        parameter Modelica.SIunits.Time activationTime=0.0
          "Time when controller is switched on"
          annotation (Dialog(group="Activation"));

        outer parameter Boolean use_kInput=false
          "True, if proportional gain is defined externally" annotation (Dialog(
            enable=not (controllerType == "I"),
            tab="Advanced",
            group="Gain Scheduling"));
        outer parameter Boolean use_kiInput=false
          "True, if integral gain is defined externally" annotation (Dialog(
            enable=not (controllerType == "P"),
            tab="Advanced",
            group="Gain Scheduling"));
        outer parameter Boolean use_bInput=false
          "True, if setpoint weight is defined externally"
          annotation (Dialog(tab="Advanced", group="Gain Scheduling"));
        outer parameter Boolean invertFeedback=false
          "true, if feedback Modelica.SIUnitsgnal is inverted"
          annotation (dialog(group="Settings"));
        outer parameter Boolean use_setpointWeighting=true "Available for PI"
          annotation (Dialog(
            enable=(controllerType == "PI") or (controllerType == "P"),
            tab="Advanced",
            group="Setpoint weighting"));
        /*********************INITIALIZATION****************/
        outer parameter String initType="initialOutput" "Type of initialization"
          annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
            dialog(enable=controllerType <> "P", group="Initialization"));
        parameter Real yInitial=0 "Initial output of controller"
          annotation (dialog(enable=controllerType <> "P", group="Initialization"));
        parameter Real[n] xInitial=zeros(n) "Initial states of controller"
          annotation (dialog(enable=(controllerType == "I") or (controllerType == "PI"),
              group="Initialization"));
        /*********************COMPONENTS********************/
      protected
        MISO_Controller.miso_decentralcontroller_outer[n] pi(
          each n=n,
          KP=KP,
          KI=KI,
          B=B,
          each activationTime = activationTime,
          yInitial = zeros(n)) annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));

      equation
        for outputs in 1:n loop
          // Connect the input Signals and the Output signal
          // Setpoint, n Signals
          connect(u_s, pi[outputs].u_s);
          // Measurement, n Signals
          connect(u_m, pi[outputs].u_m);
          // Anti-Windup, 1 Signal per output
          connect(u_a[outputs],pi[outputs].u_a);
          // Output
          connect(pi[outputs].y, y[outputs]);
          // Connect the external Parameter
          // Proportional gain
          if use_kInput then
            connect(KP_in[outputs], pi[outputs].KP_in);
          end if;
          // Integral gain
          if use_kiInput then
            connect(KI_in[outputs], pi[outputs].KI_in);
          end if;
          // Setpoint-Weight
          if use_bInput then
            connect(B_in[outputs], pi[outputs].B_in);
          end if;
          // Active Input
          if use_activeInput then
            connect(activeInput, pi[outputs].activeInput);
          end if;
        end for;

        annotation (
          Documentation(info="<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-52,64},{40,-50}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="K")}),
          Diagram(coordinateSystem(preserveAspectRatio=false)));
      end mimo_decentralcontroller_outer;

      block mimo_decentralcontroller_single
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
                Modelica.Blocks.Interfaces.RealInput u_a[n] "Connector of anti-windup inputs"
          annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={60,-106}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={58,-94})));
        Modelica.Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals"
          annotation (Placement(transformation(extent={{80,-20},{120,20}}),
              iconTransformation(extent={{80,-20},{120,20}})));
        // External
        Modelica.Blocks.Interfaces.RealInput[n,n] B_in if use_bInput "External defined B"
          annotation (Placement(transformation(extent={{-122,-20},{-82,20}}),
              iconTransformation(extent={{-122,-20},{-82,20}})));
        Modelica.Blocks.Interfaces.RealInput[n,n] KI_in if
                                                          use_kiInput "External defined KI."
          annotation (Placement(transformation(extent={{-130,20},{-90,60}}),
              iconTransformation(extent={{-122,20},{-82,60}})));
        Modelica.Blocks.Interfaces.RealInput[n,n] KP_in if use_kInput
                                                                     "External defined KP."
          annotation (Placement(transformation(extent={{-130,60},{-90,100}}),
              iconTransformation(extent={{-122,60},{-82,100}})));
        Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput "activeInput" annotation (
            Placement(transformation(extent={{-122,-100},{-82,-60}}),
              iconTransformation(extent={{-122,-100},{-82,-60}})));

        /*********************ACTIVATION********************/
      public
        inner parameter Boolean use_activeInput=false
          "True, if controller is switched on/off externally"
          annotation (Dialog(group="Activation"));
        inner parameter Boolean use_y_notActive=false
          "NOT IMPLEMENTED"
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
      protected
        MISO_Controller.miso_decentralcontroller_outer[n] pi(
          each n=n,
          KP=KP,
          KI=KI,
          B=B,
          each activationTime = activationTime) annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));

      equation
        for outputs in 1:n loop
          // Connect the input Signals and the Output signal
          // Setpoint, n Signals
          connect(u_s, pi[outputs].u_s);
          // Measurement, n Signals
          connect(u_m, pi[outputs].u_m);
          // Anti-Windup, 1 Signal per output
          connect(u_a[outputs],pi[outputs].u_a);
          // Output
          connect(pi[outputs].y, y[outputs]);
          // Connect the external Parameter
          // Proportional gain
          if use_kInput then
            connect(KP_in[outputs], pi[outputs].KP_in);
          end if;
          // Integral gain
          if use_kiInput then
            connect(KI_in[outputs], pi[outputs].KI_in);
          end if;
          // Setpoint-Weight
          if use_bInput then
            connect(B_in[outputs], pi[outputs].B_in);
          end if;
          // Active Input
          if use_activeInput then
            connect(activeInput, pi[outputs].activeInput);
          end if;
        end for;

        annotation (
          Documentation(info="<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-52,64},{40,-50}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="K")}),
          Diagram(coordinateSystem(preserveAspectRatio=false)));
      end mimo_decentralcontroller_single;

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
</html>"),Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-52,64},{40,-50}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="K")}),
          Diagram(coordinateSystem(preserveAspectRatio=false)));
      end mimo_decentralcontroller_2_old;
    end MIMO_Controller;

    package MIMO_Decoupler "Consists of Decoupler"

      model mimo_decoupler_outer "Decoupler for MIMO Systems"
        extends Modelica.Blocks.Icons.Block;
        outer parameter Boolean use_dInput = false "Use external inpput";
        outer parameter Integer n(min=1) "Number of inputs";
        outer parameter Real[n,n] D "Decoupling Matrix";
        outer parameter Real[n] ymax "Maximum Output";
        outer parameter Real[n] ymin "Minimum Output";
        outer parameter Real[n] offset "Offset";
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
                Modelica.Blocks.Interfaces.RealInput D_in[n,n] if use_dInput
          "Connector of Real input signals" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,108}), iconTransformation(extent={{-20,-20},{20,20}},
              rotation=270,
              origin={2,94})));
      protected
        Modelica.Blocks.Sources.Constant[n,n] D_in_(k = D) if not use_dInput;
        //Modelica.Blocks.Math.MatrixGain Gain(K = D_int.k);
        Modelica.Blocks.Interfaces.RealInput[n,n] D_int;
        Modelica.Blocks.Nonlinear.Limiter limiter[n](uMax=ymax, uMin=ymin)
          annotation (Placement(transformation(extent={{2,-10},{22,10}})));
      equation
        connect(D_in_.y,D_int);
        connect(D_in, D_int);
        // Output of the Limiter is systems output
        connect(y, limiter.y);
        // Input of the matrix gain is the systems input
        //Gain.u = u;
        // Output of the Matrix gain is the limiter input, additional offset
        limiter.u = D_int*u + offset;
        // Anti-Windup is difference between limiter Input and Output
        y_a = limiter.y - limiter.u;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-50,60},{42,-54}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="D")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end mimo_decoupler_outer;

      model mimo_decoupler_single "Decoupler for MIMO Systems"
        extends Modelica.Blocks.Icons.Block;
        parameter Boolean use_dInput = false "Use external inpput";
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
      protected
        Modelica.Blocks.Sources.Constant[n,n] D_in_(k = D) if not use_dInput;
        //Modelica.Blocks.Math.MatrixGain Gain(K = D_int.k);
        Modelica.Blocks.Interfaces.RealInput D_in[n,n] if use_dInput
          "Connector of Real input signals" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,108}), iconTransformation(extent={{-20,-20},{20,20}},
              rotation=270,
              origin={2,94})));
        Modelica.Blocks.Interfaces.RealInput[n,n] D_int;
      equation
        connect(D_in_.y,D_int);
        connect(D_in, D_int);
        // Output of the Limiter is systems output
        connect(y, limiter.y);
        // Input of the matrix gain is the systems input
        //Gain.u = u;
        // Output of the Matrix gain is the limiter input
        limiter.u = D_int*u;
        // Anti-Windup is difference between limiter Input and Output
        y_a = limiter.y - limiter.u;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-50,60},{42,-54}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="D")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end mimo_decoupler_single;
    end MIMO_Decoupler;

    model Multivariable_Controller
      "Consits of a controller and a decoupler"
      /**************CONNECTORS******************/
      // Signals
    public
      Modelica.Blocks.Interfaces.RealInput[n] u_s "Setpoint signal"
        annotation (Placement(transformation(extent={{-120,-100},{-80,-60}}),
            iconTransformation(extent={{-120,-100},{-80,-60}})));
      Modelica.Blocks.Interfaces.RealInput[n] u_m "Measurement signal" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-38,-109})));
      Modelica.Blocks.Interfaces.RealOutput[n] y "Output signal"
        annotation (Placement(transformation(extent={{96,-10},{116,10}})));
      // External Control
      Modelica.Blocks.Interfaces.RealInput[n,n] KP_in if use_kInput "KP from external"
        annotation (Placement(transformation(extent={{-120,60},{-80,100}}),
            iconTransformation(extent={{-120,60},{-80,100}})));
      Modelica.Blocks.Interfaces.RealInput[n,n] KI_in if use_kiInput "KI from external"
        annotation (Placement(transformation(extent={{-120,20},{-80,60}}),
            iconTransformation(extent={{-120,20},{-80,60}})));
      Modelica.Blocks.Interfaces.RealInput[n,n] B_in if use_bInput "B from external"
        annotation (Placement(transformation(extent={{-120,-20},{-80,20}}),
            iconTransformation(extent={{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealInput[n,n] D_in if use_dInput "D from external" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-100,-30}), iconTransformation(extent={{-120,-60},{-80,-20}})));
            // Activation
      Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput
        "Activate controller external"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,110})));
      /*************PARAMETER*******************/
    public
      inner parameter Integer n(min=1) "Number of inputs (= number of outputs)";

      inner parameter String initType="initialOutput" "Type of initialization"
        annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
          dialog(enable=controllerType <> "P", group="Initialization"));
      inner parameter String controllerType="PI" "Controller Type" annotation (
          choices(
          choice="P" "P controller",
          choice="PI" "PI controller",
          choice="I" "I controller"), dialog(group="Settings"));
      inner parameter Real[n,n] KP = identity(n) "Proportional Gain" annotation(Dialog(enable = (not use_kInput) and (not ControllerType == "I")));
      inner parameter Real[n,n] KI = identity(n) "Integral Gain"
                                                                annotation(Dialog(enable = (not use_kiInput) and (not ControllerType == "P")));
      inner parameter Real[n,n] B = identity(n) "Set Point Weights"
                                                                   annotation(Dialog(enable = (not use_bInput) and ( use_setpointWeighting)));
      inner parameter Real[n,n] D = identity(n) "Decoupler"
                                                           annotation(Dialog(enable = not use_dInput));
      inner parameter Real[n] ymax = fill(100,n) "Upper bound of the output";
      inner parameter Real[n] ymin = -ymax "Lower bound of the output";
      inner parameter Real[n] offset=zeros(n)
        "Operating point, added to proportional output";
      inner parameter Boolean use_activeInput=false
        "True, if controller is switched on/off externally"
        annotation (Dialog(group="Activation"));
      parameter Modelica.SIunits.Time activationTime=0.0
        "Time when controller is switched on"
        annotation (Dialog(group="Activation", enable = not use_activeInput));

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
          inner parameter Boolean use_dInput=false
        "True, if decoupler is defined externally"
        annotation (Dialog(tab="Advanced", group="Gain Scheduling"));
      inner parameter Boolean invertFeedback=false
        "true, if feedback Modelica.SIUnitsgnal is inverted"
        annotation (dialog(group="Settings"));
      inner parameter Boolean use_setpointWeighting=true "Available for PI"
        annotation (Dialog(
          enable=(controllerType == "PI") or (controllerType == "P"),
          tab="Advanced",
          group="Setpoint weighting"));


      /*************COMPONENTS*****************/
    protected
      MIMO_Controller.mimo_decentralcontroller_outer mimo_decentralcontroller1(
          activationTime=activationTime)
        annotation (Placement(transformation(extent={{-42,-10},{-22,10}})));
      MIMO_Decoupler.mimo_decoupler_outer mimo_decoupler1   annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        /*************EQUATION*****************/


    equation
      // Input Signals
      connect(u_s, mimo_decentralcontroller1.u_s);
      connect(u_m, mimo_decentralcontroller1.u_m);
      // Connect controller and decoupler
      mimo_decentralcontroller1.y  =  mimo_decoupler1.u
        annotation (Line(points={{-22,0},{0,0},{22,0}}, color={0,0,127}));
        // Internal connection for Anti-Windup
      connect(mimo_decoupler1.y_a, mimo_decentralcontroller1.u_a) annotation (Line(
            points={{32,-10},{32,-24},{-26.2,-24},{-26.2,-9.4}}, color={0,0,127}));
            // Connect Output
            mimo_decoupler1.y =  y;
      // Connect External variables
      // Proportional gain
        if use_kInput then
          connect(KP_in,mimo_decentralcontroller1.KP_in);
        end if;
        // Integral gain
        if use_kiInput then
          connect(KI_in, mimo_decentralcontroller1.KI_in);
        end if;
        // Setpoint-Weight
        if use_bInput then
          connect(B_in, mimo_decentralcontroller1.B_in);
        end if;
        // Decoupler
        if use_dInput then
          connect(D_in, mimo_decoupler1.D_in);
        end if;
        // Active Input
        if use_activeInput then
          connect(activeInput, mimo_decentralcontroller1.activeInput);
        end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
              Text(
              extent={{-66,80},{72,-72}},
              lineColor={0,0,0},
              textString="K D",
              textStyle={TextStyle.Bold})}),                         Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Multivariable_Controller;
  end Controller;

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
        use_activeInput=false,
        use_y_notActive=false,
        activationTime=30)
        annotation (Placement(transformation(extent={{-10,8},{2,20}})));
      Modelica.Blocks.Sources.Step u_r(
        height=1,
        offset=0,
        startTime=2)
        annotation (Placement(transformation(extent={{-76,4},{-56,24}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1)
        annotation (Placement(transformation(extent={{26,8},{46,28}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={38,-22})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=2)
        annotation (Placement(transformation(extent={{-76,28},{-56,48}})));
      Modelica.Blocks.Continuous.SecondOrder secondOrder(
        k=2,
        w=300,
        D=0.9) annotation (Placement(transformation(extent={{68,-18},{88,2}})));
    equation
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
      connect(pIController.u_m, secondOrder.y) annotation (Line(points={{-6.6,
              8.2},{42,8.2},{42,-8},{89,-8}}, color={0,0,127}));
      connect(limiter.y, secondOrder.u) annotation (Line(points={{47,18},{56,18},
              {56,-8},{66,-8}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_SISOCN;

    model Test_SISOCN_SW "Tester for SISO system"

      PI_SP.PIController_sw pIController(
        controllerType="PI",
        invertFeedback=false,
        integralParameterType="ki",
        k=1,
        ki=0.1,
        initType="zeroIntegralState",
        use_setpointWeighting=true,
        use_activeInput=false,
        use_y_notActive=false,
        activationTime=30)
        annotation (Placement(transformation(extent={{-10,8},{2,20}})));
      Modelica.Blocks.Sources.Step u_r(
        height=1,
        offset=0,
        startTime=2)
        annotation (Placement(transformation(extent={{-76,4},{-56,24}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1)
        annotation (Placement(transformation(extent={{26,8},{46,28}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={38,-22})));
      Modelica.Blocks.Continuous.SecondOrder secondOrder(
        k=2,
        w=300,
        D=0.9) annotation (Placement(transformation(extent={{62,12},{82,32}})));
    equation
      connect(pIController.y, limiter.u)
        annotation (Line(points={{2.4,14},{2.4,18},{24,18}}, color={0,0,127}));
      connect(add.u2, pIController.y) annotation (Line(points={{32,-10},{20,-10},
              {20,14},{2.4,14}}, color={0,0,127}));
      connect(add.u1, limiter.y)
        annotation (Line(points={{44,-10},{47,-10},{47,18}}, color={0,0,127}));
      connect(pIController.u_s, u_r.y) annotation (Line(points={{-9.6,14},{-32,
              14},{-55,14}}, color={0,0,127}));
      connect(pIController.u_m, secondOrder.y) annotation (Line(points={{-4,8.2},
              {42,8.2},{42,22},{83,22}},color={0,0,127}));
      connect(limiter.y, secondOrder.u) annotation (Line(points={{47,18},{56,18},
              {56,22},{60,22}},
                            color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_SISOCN_SW;

    model Test_MISOCN "Tester for MISO controller"
       /*******************COMPONENTS************/
      Controller.MISO_Controller.miso_decentralcontroller_single
        miso_decentralcontroller1(
        n=2,
        B={0,0},
        KI={0.1,0},
        KP={1,0},
        controllerType="PI",
        use_activeInput=true,
        use_kInput=true)
                  annotation (Placement(transformation(extent={{12,22},{32,42}})));
      Modelica.Blocks.Sources.Step step(
        height=-1,
        offset=10,
        startTime=200)
        annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(        uMin=-1, uMax=1)
        annotation (Placement(transformation(extent={{50,22},{70,42}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={60,0})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder(
        T=1,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=0,
        k=1) annotation (Placement(transformation(extent={{98,22},{118,42}})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
      Modelica.Blocks.Continuous.Integrator integrator(k=0.1)
        annotation (Placement(transformation(extent={{136,22},{156,42}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=true, startTime=350)
        annotation (Placement(transformation(extent={{-72,-42},{-52,-22}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=5)
        annotation (Placement(transformation(extent={{2,70},{22,90}})));
      Modelica.Blocks.Sources.RealExpression realExpression1
        annotation (Placement(transformation(extent={{-38,58},{-18,78}})));
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
      connect(limiter.y, firstOrder.u)
        annotation (Line(points={{71,32},{96,32}}, color={0,0,127}));
      connect(const.y, miso_decentralcontroller1.u_s[2]) annotation (Line(points={{-59,
              10},{-24,10},{-24,30.6},{11.6,30.6}}, color={0,0,127}));
      connect(firstOrder.y, integrator.u)
        annotation (Line(points={{119,32},{114,32},{134,32}}, color={0,0,127}));
      connect(integrator.y, miso_decentralcontroller1.u_m[1]) annotation (Line(
            points={{157,32},{162,32},{162,-34},{162,-46},{16.2,-46},{16.2,21.8}},
            color={0,0,127}));
      connect(miso_decentralcontroller1.activeInput, booleanStep.y) annotation (
          Line(points={{11.8,24.2},{-2,24.2},{-2,-32},{-51,-32}}, color={255,0,255}));
      connect(step.y, miso_decentralcontroller1.u_s[1]) annotation (Line(points=
             {{-59,40},{-24,40},{-24,29},{11.6,29}}, color={0,0,127}));
      connect(miso_decentralcontroller1.KP_in[1], realExpression.y) annotation (
         Line(points={{11.6,39.9},{11.6,80},{23,80}}, color={0,0,127}));
      connect(miso_decentralcontroller1.KP_in[2], realExpression1.y)
        annotation (Line(points={{11.6,41.3},{22,41.3},{22,68},{-17,68}}, color=
             {0,0,127}));
      connect(const.y, miso_decentralcontroller1.u_m[2]) annotation (Line(
            points={{-59,10},{-30,10},{-30,8},{16.2,8},{16.2,23.8}}, color={0,0,
              127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MISOCN;

    model Test_MIMOCN "Tester for MIMO Controller"

      Controller.MIMO_Controller.mimo_decentralcontroller_outer
        mimo_decentralcontroller1(
        n=2,
        controllerType="PI",
        B={{0,0},{0,0}},
        KP={{5,0},{0,1}},
        KI={{0.5,0},{0,0.1}},
        use_kiInput=true,
        use_kInput=true,
        use_bInput=true,
        activationTime=30,
        use_activeInput=false)
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Blocks.Sources.Step step(
        offset=5,
        startTime=250,
        height=-2)
        annotation (Placement(transformation(extent={{-90,-40},{-70,-20}})));
      Modelica.Blocks.Sources.Step step1(
        height=-3,
        startTime=100,
        offset=2)
        annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
      Controller.MIMO_Decoupler.mimo_decoupler_outer mimo_decoupler(
        n=2,
        D=identity(2),
        ymin={-1,-2},
        ymax={0.8,0.3})
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Transferfunctions.mimo_transferfunction mimo_transferfunction(
        n=2,
        delay={{0,0},{0,0}},
        num={{{5},{1e-10}},{{1e-10},{10}}},
        den={{{10,1},{1e-10,1}},{{1e-10,1},{5,1}}})
        annotation (Placement(transformation(extent={{54,-12},{74,8}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=false, startTime=120)
        annotation (Placement(transformation(extent={{-64,-70},{-44,-50}})));
      Modelica.Blocks.Sources.RealExpression[2,2] KI(y={{0.5,0},{0,0.8}})
        annotation (Placement(transformation(extent={{-86,54},{-66,74}})));
      Modelica.Blocks.Sources.RealExpression[2,2] KP(y={{1,0},{0,2}})
        annotation (Placement(transformation(extent={{-86,70},{-66,90}})));
      Modelica.Blocks.Sources.RealExpression[2,2] B(y={{0.5,0},{0,1}})
        annotation (Placement(transformation(extent={{-86,38},{-66,58}})));
    equation
      connect(step1.y, mimo_decentralcontroller1.u_s[2]) annotation (Line(points={{-69,0},
              {-40,0},{-40,-3},{-12.2,-3}},  color={0,0,127}));
      connect(step.y, mimo_decentralcontroller1.u_s[1]) annotation (Line(points={{-69,-30},
              {-42,-30},{-42,-5},{-12.2,-5}},  color={0,0,127}));
      connect(mimo_decentralcontroller1.y, mimo_decoupler.u)
        annotation (Line(points={{8,0},{20,0}}, color={0,0,127}));
      connect(mimo_decoupler.y_a, mimo_decentralcontroller1.u_a) annotation (Line(
            points={{30,-10},{30,-24},{3.8,-24},{3.8,-9.4}}, color={0,0,127}));
      connect(mimo_decoupler.y, mimo_transferfunction.u)
        annotation (Line(points={{40,0},{48,0},{48,-2},{54,-2}}, color={0,0,127}));
      connect(mimo_transferfunction.y, mimo_decentralcontroller1.u_m) annotation (
          Line(points={{74,-2},{88,-2},{88,-38},{-8,-38},{-8,-9.6}}, color={0,0,127}));
      connect(mimo_decentralcontroller1.activeInput, booleanStep.y) annotation (
          Line(points={{-12.2,-8},{-28,-8},{-28,-60},{-43,-60}}, color={255,0,255}));
      connect(mimo_decentralcontroller1.KI_in, KI.y) annotation (Line(points={{
              -12.2,4},{-28,4},{-28,64},{-65,64}}, color={0,0,127}));
      connect(mimo_decentralcontroller1.KP_in, KP.y) annotation (Line(points={{
              -12.2,8},{-18,8},{-18,80},{-22,80},{-65,80}}, color={0,0,127}));
      connect(mimo_decentralcontroller1.B_in, B.y) annotation (Line(points={{
              -12.2,0},{-38,0},{-38,48},{-65,48}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_MIMOCN;

    model Test_Multivariable

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

      Transferfunctions.mimo_transferfunction Rosenbrocks_System(
        n=2,
        num={{{1},{2}},{{1},{1}}},
        den={{{1,1},{1,3}},{{1,1},{1,1}}},
        delay=fill(
                0,
                2,
                2))
        annotation (Placement(transformation(extent={{40,0},{60,20}})));

      Modelica.Blocks.Sources.Step step(
        startTime=1,
        offset=0,
        height=8)
        annotation (Placement(transformation(extent={{-92,-12},{-72,8}})));
      Modelica.Blocks.Sources.Step step1(
        startTime=5,
        offset=0,
        height=9)
        annotation (Placement(transformation(extent={{-94,-66},{-74,-46}})));
      Controller.Multivariable_Controller multivariable_Controller(
        n=2,
        controllerType="PI",
        use_kiInput=true,
        use_bInput=true,
        use_dInput=true,
        use_activeInput=true,
        activationTime=0,
        ymax={20,10},
        ymin={5,2},
        offset={15,6},
        use_kInput=true,
        initType="zeroIntegralState")
        annotation (Placement(transformation(extent={{-12,0},{8,20}})));
      Modelica.Blocks.Sources.RealExpression[2,2] KI(y={{0.852,0},{0,0.852}})
        annotation (Placement(transformation(extent={{-100,42},{-80,62}})));
      Modelica.Blocks.Sources.RealExpression[2,2] KP(y={{0.725,0},{0,0.725}})
        annotation (Placement(transformation(extent={{-100,54},{-80,74}})));
      Modelica.Blocks.Sources.RealExpression[2,2] B(y={{0,0},{0,0}})
        annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
      Modelica.Blocks.Sources.RealExpression[2,2] D(y={{1,0},{0,1}})
        annotation (Placement(transformation(extent={{-100,16},{-80,36}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=30)
        annotation (Placement(transformation(extent={{-44,70},{-24,90}})));
    equation
      for inputs in 1:n loop
      end for;
      connect(Rosenbrocks_System.u, multivariable_Controller.y)
        annotation (Line(points={{40,10},{24,10},{8.6,10}}, color={0,0,127}));
      connect(multivariable_Controller.u_m, Rosenbrocks_System.y) annotation (
          Line(points={{-5.8,-0.9},{-5.8,-32},{76,-32},{76,10},{60,10}}, color=
              {0,0,127}));
      connect(multivariable_Controller.u_s[1], step1.y) annotation (Line(points
            ={{-12,1},{-42,1},{-42,-56},{-73,-56}}, color={0,0,127}));
      connect(multivariable_Controller.u_s[2], step.y) annotation (Line(points=
              {{-12,3},{-42,3},{-42,-2},{-71,-2}}, color={0,0,127}));
      connect(multivariable_Controller.KP_in, KP.y) annotation (Line(points={{
              -12,18},{-14,18},{-14,64},{-79,64}}, color={0,0,127}));
      connect(multivariable_Controller.KI_in, KI.y) annotation (Line(points={{
              -12,14},{-28,14},{-28,52},{-79,52}}, color={0,0,127}));
      connect(multivariable_Controller.B_in, B.y) annotation (Line(points={{-12,
              10},{-46,10},{-46,40},{-79,40}}, color={0,0,127}));
      connect(multivariable_Controller.D_in, D.y) annotation (Line(points={{-12,
              6},{-46,6},{-46,26},{-79,26}}, color={0,0,127}));
      connect(multivariable_Controller.activeInput, booleanStep.y) annotation (
          Line(points={{-2,21},{-2,21},{-2,80},{-23,80}}, color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_Multivariable;

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
      Controller.MIMO_Controller.mimo_decentralcontroller_outer
        Decentral_Controller(
        n=2,
        KP=[0.725,1e-10; 1e-10,0.725],
        KI=[0.852,1e-10; 1e-10,0.852],
        B=[0,1e-10; 1e-10,0])
        annotation (Placement(transformation(extent={{-36,0},{-16,20}})));

      Controller.MIMO_Decoupler.mimo_decoupler_outer Decoupler(
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

    model Test_Decoupler

      Controller.MIMO_Decoupler.mimo_decoupler_single mimo_decoupler_single(
        n=2,
        D=identity(2),
        ymax={1,1},
        ymin={-2,0},
        use_dInput=true)
        annotation (Placement(transformation(extent={{-32,12},{-12,32}})));
      Modelica.Blocks.Sources.RealExpression[2,2] D(y={{1,2},{-1,1}})
        annotation (Placement(transformation(extent={{-88,50},{-68,70}})));
      Modelica.Blocks.Sources.RealExpression[2] D1(y={1,1})
        annotation (Placement(transformation(extent={{-86,12},{-66,32}})));
    equation
      connect(mimo_decoupler_single.D_in, D.y) annotation (Line(points={{-21.8,
              31.4},{-21.8,32},{-22,32},{-22,60},{-67,60}},
                                                      color={0,0,127}));
      connect(mimo_decoupler_single.u, D1.y) annotation (Line(points={{-32,22},
              {-54,22},{-54,26},{-65,26},{-65,22}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Test_Decoupler;
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
      Controller.MIMO_Controller.mimo_decentralcontroller_outer
        Decentral_Controller(
        n=n,
        B=b,
        KI=ki,
        KP=kp) annotation (Placement(transformation(extent={{-36,0},{-16,20}})));

      Controller.MIMO_Decoupler.mimo_decoupler_outer Decoupler(
        n=n,
        D=d,
        ymax=ymax,
        ymin=ymin)
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
