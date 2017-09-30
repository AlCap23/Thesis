package Masterthesis "Includes all necessary models"

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
        parameter String initType="initialOutput" "Type of initialization"
          annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
            dialog(enable=controllerType <> "P", group="Initialization"));
        parameter Real yInitial=0 "Initial output of controller"
          annotation (dialog(enable=controllerType <> "P", group="Initialization"));
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
          each yInitial= 0,
          each use_activeInput=use_activeInput,
          each use_y_notActive=use_y_notActive,
          each activationTime=activationTime,
          each invertFeedback=invertFeedback,
          each use_kInput=use_kInput,
          each use_kiInput=use_kiInput,
          each use_bInput=use_bInput,
          each initType="zeroIntegralState")
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
        parameter String initType="initialOutput" "Type of initialization"
          annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
            dialog(enable=controllerType <> "P", group="Initialization"));
      protected
        Real[n] yInitial=zeros(n) "Initial output of controller"
          annotation (dialog(enable=controllerType <> "P", group="Initialization"));

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
      end mimo_decentralcontroller_outer;
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
        outer parameter Real[n] yInitial "Initial output of controller";
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
        // External decoupler
      protected
        Modelica.Blocks.Sources.Constant[n,n] D_in_(k = D) if not use_dInput;
        Modelica.Blocks.Interfaces.RealInput[n,n] D_int;
        Modelica.Blocks.Nonlinear.Limiter limiter[n](uMax=ymax, uMin=ymin)
          annotation (Placement(transformation(extent={{2,-10},{22,10}})));
          /***********INITIAL******************/

      /*************EQUATION*******************/
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
          annotation (choices(choice="zeroIntegralState", choice="initialOutput"),
            dialog(enable=controllerType <> "P", group="Initialization"),
                    Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
                extent={{-50,60},{42,-54}},
                lineColor={0,0,0},
                textStyle={TextStyle.Bold},
                textString="D")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end mimo_decoupler_outer;
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

      parameter String initType="initialOutput" "Type of initialization"
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
      inner parameter Real[n] yInitial = zeros(n) "Initial Output of Decoupler" annotation(dialog(enable=(not controllerType=="P"), group = "Initialization"));
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
  annotation (
    uses(Modelica(version = "3.2.2")));
end Masterthesis;

package ModelicaServices
  "ModelicaServices (Default implementation) - Models and functions used in the Modelica Standard Library requiring a tool specific implementation"
extends Modelica.Icons.Package;

package Machine

  final constant Real eps=1.e-15 "Biggest number such that 1.0 + eps = 1.0";

  final constant Real small=1.e-60
    "Smallest number such that small and -small are representable on the machine";
  annotation (Documentation(info="<html>
<p>
Package in which processor specific constants are defined that are needed
by numerical algorithms. Typically these constants are not directly used,
but indirectly via the alias definition in
<a href=\"modelica://Modelica.Constants\">Modelica.Constants</a>.
</p>
</html>"));
end Machine;
annotation (
  Protection(access=Access.hide),
  preferredView="info",
  version="3.2.2",
  versionBuild=0,
  versionDate="2016-01-15",
  dateModified = "2016-01-15 08:44:41Z",
  revisionId="$Id:: package.mo 9141 2016-03-03 19:26:06Z #$",
  uses(Modelica(version="3.2.2")),
  conversion(
    noneFromVersion="1.0",
    noneFromVersion="1.1",
    noneFromVersion="1.2",
    noneFromVersion="3.2.1"),
  Documentation(info="<html>
<p>
This package contains a set of functions and models to be used in the
Modelica Standard Library that requires a tool specific implementation.
These are:
</p>

<ul>
<li> <a href=\"modelica://ModelicaServices.Animation.Shape\">Shape</a>
     provides a 3-dim. visualization of elementary
     mechanical objects. It is used in
<a href=\"modelica://Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape\">Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape</a>
     via inheritance.</li>

<li> <a href=\"modelica://ModelicaServices.Animation.Surface\">Surface</a>
     provides a 3-dim. visualization of
     moveable parameterized surface. It is used in
<a href=\"modelica://Modelica.Mechanics.MultiBody.Visualizers.Advanced.Surface\">Modelica.Mechanics.MultiBody.Visualizers.Advanced.Surface</a>
     via inheritance.</li>

<li> <a href=\"modelica://ModelicaServices.ExternalReferences.loadResource\">loadResource</a>
     provides a function to return the absolute path name of an URI or a local file name. It is used in
<a href=\"modelica://Modelica.Utilities.Files.loadResource\">Modelica.Utilities.Files.loadResource</a>
     via inheritance.</li>

<li> <a href=\"modelica://ModelicaServices.Machine\">ModelicaServices.Machine</a>
     provides a package of machine constants. It is used in
<a href=\"modelica://Modelica.Constants\">Modelica.Constants</a>.</li>

<li> <a href=\"modelica://ModelicaServices.Types.SolverMethod\">Types.SolverMethod</a>
     provides a string defining the integration method to solve differential equations in
     a clocked discretized continuous-time partition (see Modelica 3.3 language specification).
     It is not yet used in the Modelica Standard Library, but in the Modelica_Synchronous library
     that provides convenience blocks for the clock operators of Modelica version &ge; 3.3.</li>
</ul>

<p>
This implementation is targeted for Dymola.
</p>

<p>
<b>Licensed by DLR and Dassault Syst&egrave;mes AB under the Modelica License 2</b><br>
Copyright &copy; 2009-2016, DLR and Dassault Syst&egrave;mes AB.
</p>

<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>

</html>"));
end ModelicaServices;

package TIL "TIL-Library"
  import PI = Modelica.Constants.pi;
  import SI = Modelica.SIunits;
  import NonSI = Modelica.SIunits.Conversions.NonSIunits;
  import g = Modelica.Constants.g_n;

  model SystemInformationManager "System Information Manager"

  /************************ VLEFluids ****************************/
  protected
    parameter Integer nVLEFluids(min=0) = 3
      "|VLE fluids|Number of VLE fluid types"
      annotation(choices(
        choice=0 "no VLE fluids",
        choice=1 "1 VLE fluid",
        choice=2 "2 VLE fluids",
        choice=3 "3 VLE fluids"));
  public
   replaceable parameter TILMedia.VLEFluidTypes.TILMedia_CO2
     vleFluidType1 constrainedby TILMedia.VLEFluidTypes.BaseVLEFluid(final ID=1)
      "VLE fluid type 1"
                        annotation(choicesAllMatching=true,
     Dialog(enable=(nVLEFluids>0),group="VLE fluids"),
     Placement(transformation(extent={{0,80},{20,100}})));

   replaceable parameter TILMedia.VLEFluidTypes.TILMedia_CO2
     vleFluidType2 constrainedby TILMedia.VLEFluidTypes.BaseVLEFluid(final ID=2)
      "VLE fluid type 2"
                        annotation(choicesAllMatching=true,
     Dialog(enable=(nVLEFluids>1),group="VLE fluids"),
     Placement(transformation(extent={{40,80},{60,100}})));

   replaceable parameter TILMedia.VLEFluidTypes.TILMedia_CO2
     vleFluidType3 constrainedby TILMedia.VLEFluidTypes.BaseVLEFluid(final ID=3)
      "VLE fluid type 3"
                        annotation(choicesAllMatching=true,
     Dialog(enable=(nVLEFluids>2),group="VLE fluids"),
     Placement(transformation(extent={{80,80},{100,100}})));

  /************************ Gases ****************************/
  protected
    parameter Integer nGases(min=0) = 3 "|Gases|Number of gas types"
      annotation(choices(
        choice=0 "no Gases",
        choice=1 "1 Gas",
        choice=2 "2 Gases",
        choice=3 "3 Gases"));

  public
   replaceable parameter TILMedia.GasTypes.VDI4670_MoistAir
     gasType1 constrainedby TILMedia.GasTypes.BaseGas(final ID=1) "Gas type 1"
                  annotation(choicesAllMatching=true,
     Dialog(enable=(nGases>0),group="Gases"),
     Placement(transformation(extent={{0,80},{20,100}})));

   replaceable parameter TILMedia.GasTypes.VDI4670_MoistAir
     gasType2 constrainedby TILMedia.GasTypes.BaseGas(final ID=2) "Gas type 2"
                  annotation(choicesAllMatching=true,
     Dialog(enable=(nGases>1),group="Gases"),
     Placement(transformation(extent={{0,80},{20,100}})));

   replaceable parameter TILMedia.GasTypes.VDI4670_MoistAir
     gasType3 constrainedby TILMedia.GasTypes.BaseGas(final ID=3) "Gas type 3"
                  annotation(choicesAllMatching=true,
     Dialog(enable=(nGases>2),group="Gases"),
     Placement(transformation(extent={{0,80},{20,100}})));

  /************************ Liquids ****************************/
  protected
    parameter Integer nLiquids(
      min=0) = 3 "|Liquids|Number of liquids"
      annotation(choices(
        choice=0 "no Liquids",
        choice=1 "1 Liquid",
        choice=2 "2 Liquids",
        choice=3 "3 Liquids"));

  public
   replaceable parameter TILMedia.LiquidTypes.TILMedia_Water
     liquidType1 constrainedby TILMedia.LiquidTypes.BaseLiquid(final ID=1)
      "Liquid type 1"
                     annotation(choicesAllMatching=true,
     Dialog(enable=(nLiquids>0),group="Liquids"),
     Placement(transformation(extent={{0,80},{20,100}})));

   replaceable parameter TILMedia.LiquidTypes.TILMedia_Water
     liquidType2 constrainedby TILMedia.LiquidTypes.BaseLiquid(final ID=2)
      "Liquid type 2"
                     annotation(choicesAllMatching=true,
     Dialog(enable=(nLiquids>1),group="Liquids"),
     Placement(transformation(extent={{0,80},{20,100}})));

   replaceable parameter TILMedia.LiquidTypes.TILMedia_Water
     liquidType3 constrainedby TILMedia.LiquidTypes.BaseLiquid(final ID=3)
      "Liquid type 3"
                     annotation(choicesAllMatching=true,
     Dialog(enable=(nLiquids>2),group="Liquids"),
     Placement(transformation(extent={{0,80},{20,100}})));
  /************************ SLEMediums ****************************/
  protected
    parameter Integer nSLEMediums(
      min=0,
      max=2) = 0 "|SLEMediums|Number of SLEMediums"
      annotation(choices(
        choice=0 "no SLEMediums",
        choice=1 "1 SLEMedium",
        choice=2 "2 SLEMediums"));

    replaceable parameter TILMedia.SLEMediumTypes.TILMedia_AdBlue sleMediumName1 constrainedby
      TILMedia.SLEMediumTypes.BaseSLEMedium "SLEMedium name 1"
      annotation(Dialog(enable=(nSLEMediums>0),group="SLEMediums"));

    replaceable parameter TILMedia.SLEMediumTypes.TILMedia_AdBlue sleMediumName2 constrainedby
      TILMedia.SLEMediumTypes.BaseSLEMedium "SLEMedium name 2"
      annotation(Dialog(enable=(nSLEMediums>1),group="SLEMediums"));

  /************************ Parameter ****************************/
    parameter Integer nMaxPressureStates(min=0) = 10
      "Maximum number of pressure states"
      annotation(Dialog(enable=true, tab="Advanced"));
  public
      TIL.Connectors.Internals.dpdtPort[nMaxPressureStates] dpdtPort
      "Reference to different pressure states of the system"
        annotation(HideResult=true);
      TIL.Connectors.Internals.vleFluidMassPort[nVLEFluids] fluidPort
      "Reference to the volume and mass of the VLE fluid in components"
        annotation(HideResult=true);
      TIL.Connectors.Internals.liquidMassPort[nLiquids] liquidPort
      "Reference to the volume and mass of the liquid in components"
        annotation(HideResult=true);

    Modelica.Blocks.Interfaces.RealOutput cumulatedVLEFluidMassOutput[nVLEFluids]
      "Cumulated vleFluid mass"
      annotation (Placement(transformation(
          origin={-90,-40},
          extent={{-10,-10},{10,10}},
          rotation=180)));
    Modelica.Blocks.Interfaces.RealOutput cumulatedVLEFluidVolumeOutput[nVLEFluids]
      "Cumulated vleFluid volume"
      annotation (Placement(transformation(
          origin={-90,0},
          extent={{-10,-10},{10,10}},
          rotation=180)));
  protected
    Modelica.SIunits.Mass cumulatedVLEFluidMass[nVLEFluids]
      "Cumulated vleFluid mass";
    Modelica.SIunits.Volume cumulatedVLEFluidVolume[nVLEFluids]
      "Cumulated vleFluid volume";
  public
    Modelica.SIunits.Mass cumulatedLiquidMass[nLiquids] "Cumulated liquid mass";
    Modelica.SIunits.Volume cumulatedLiquidVolume[nLiquids]
      "Cumulated liquid volume";

    final parameter Boolean removeSingularity=true;
    final parameter Boolean removeNLSystems=false;
    final parameter Boolean generateEventsAtFlowReversal=true;
  equation
    assert(sum(if (dpdtPort[i].counter>1.5) then 1 else 0 for i in 1:nMaxPressureStates)==0,"At least two pressure states have the same ID");
    cumulatedVLEFluidMassOutput = cumulatedVLEFluidMass;
    cumulatedVLEFluidVolumeOutput = cumulatedVLEFluidVolume;

    cumulatedVLEFluidMass = -(fluidPort.vleFluidMass);
    cumulatedVLEFluidVolume = -(fluidPort.vleFluidVolume);

    cumulatedLiquidMass = -(liquidPort.liquidMass);
    cumulatedLiquidVolume = -(liquidPort.liquidVolume);
  //    preferedView="info",
    annotation (
      defaultComponentName="sim",
      defaultComponentPrefixes="inner",
      missingInnerMessage="Your model is using an outer \"sim\" component. 
An inner \"sim\" component is not defined. For simulation drag TIL.SystemInformationManager into 
your model.
Please also ensure that no experiment setup information is stored in your model.",
      Icon(graphics={Bitmap(
            extent={{-90,-90},{90,90}},
            imageSource=
                "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",
            fileName="modelica://TIL/Images/SIM.png")}),
      Documentation(info="<html> <br>
      The System Information Manager (SIM) is necessary in all TIL system models.
      The SIM defines the media which are used in the system.
      Furtheron the cumulated liquid and VLE fluid mass and cumulated volumes are calculated inside the SIM.
      The SIM is declared automatically as &quot;inner&quot;, so that every component model have access to it.
      <hr>
      <br>
      </html>"));
  end SystemInformationManager;

  package GasComponents "Component models using gas vapour mixtures"
    extends TIL.Internals.ClassTypes.ComponentPackage;

    package Boundaries "Boundaries"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model Boundary "Boundary element with optional inputs"
        extends TIL.GasComponents.Boundaries.BaseClasses.PartialGasBoundary;

        parameter TIL.Internals.BoundaryType boundaryType=
                       "m_flow" "Non stream variables input type"
          annotation(Dialog(group="Input Type", tab = "Potential and Flow Variables: p, m_flow, V_flow"));

        parameter Boolean use_pressureInput=false "= true, if p defined by input"
          annotation(Dialog(enable=(boundaryType=="p" or boundaryType=="p, m_flow" or boundaryType=="p, V_flow"), group = "p - Pressure", tab="Potential and Flow Variables: p, m_flow, V_flow"));
        parameter SI.Pressure pFixed = 1.013e5 "Fixed value for p"
          annotation(Dialog(enable=(not use_pressureInput and (boundaryType=="p" or boundaryType=="p, m_flow" or boundaryType=="p, V_flow")),
          tab="Potential and Flow Variables: p, m_flow, V_flow", group="p - Pressure"));

        parameter Boolean use_massFlowRateInput=false
          "= true, if m_flow defined by input"
          annotation(Dialog(enable=(boundaryType=="m_flow" or boundaryType=="p, m_flow"),
          group = "m_flow - Mass Flow Rate", tab="Potential and Flow Variables: p, m_flow, V_flow"));
        parameter SI.MassFlowRate m_flowFixed=0.0 "Fixed value for m_flow"
          annotation(Dialog(enable=(not use_massFlowRateInput and (boundaryType=="m_flow" or boundaryType=="p, m_flow")),
          tab="Potential and Flow Variables: p, m_flow, V_flow", group="m_flow - Mass Flow Rate"));

        parameter Boolean use_volumeFlowRateInput=false
          "= true, if V_flow defined by input"
          annotation(Dialog(enable=(boundaryType=="V_flow" or boundaryType=="p, V_flow"),
          tab="Potential and Flow Variables: p, m_flow, V_flow", group = "V_flow - Volume Flow Rate"));
        parameter SI.VolumeFlowRate V_flowFixed=0.0 "Fixed value for V_flow"
          annotation(Dialog(enable=(not use_volumeFlowRateInput and (boundaryType=="V_flow" or boundaryType=="p, V_flow")),
          tab="Potential and Flow Variables: p, m_flow, V_flow", group="V_flow - Volume Flow Rate"));

        Modelica.Blocks.Interfaces.RealInput p_in if
             use_pressureInput "Prescribed boundary pressure [Pa]"
          annotation (Placement(transformation(extent={{-100,70},{-80,90}}, rotation=
                  0), iconTransformation(extent={{-50,50},{-30,70}})));
        Modelica.Blocks.Interfaces.RealInput m_flow_in if
             use_massFlowRateInput "Prescribed boundary mass flow rate [kg/s]"
          annotation (Placement(transformation(extent={{-100,30},{-80,50}}, rotation=
                  0), iconTransformation(extent={{-50,10},{-30,30}})));
        Modelica.Blocks.Interfaces.RealInput V_flow_in if use_volumeFlowRateInput
          "Prescribed boundary volume flow rate [m^3/s]"
          annotation (Placement(transformation(extent={{-100,30},{-80,50}}, rotation=
                  0), iconTransformation(extent={{-50,10},{-30,30}})));

      protected
        Modelica.Blocks.Sources.Constant p_in_(k=pFixed) if not use_pressureInput;
        Modelica.Blocks.Sources.Constant m_flow_in_(k=m_flowFixed) if not use_massFlowRateInput;
        Modelica.Blocks.Sources.Constant V_flow_in_(k=V_flowFixed) if not use_volumeFlowRateInput;

      equation
        if (boundaryType=="p") then
          p = getInputs.p_in;
        elseif (boundaryType=="V_flow") then
          if (getInputs.V_flow_in > 0) then
            m_flow = getInputs.V_flow_in * gas_inStream.d;
          else
            m_flow = getInputs.V_flow_in * d;
          end if;
        else
          m_flow = getInputs.m_flow_in;
        end if;

        connect(getInputs.p_in, p_in_.y);
        connect(getInputs.m_flow_in, m_flow_in_.y);
        connect(getInputs.V_flow_in, V_flow_in_.y);

        connect(getInputs.p_in, p_in) annotation (Line(points={{-42,10},{-60,10},{-60,
                80},{-90,80}}, color={0,0,127}));
        connect(getInputs.m_flow_in, m_flow_in) annotation (Line(points={{-42,7.2},{-70,
                7.2},{-70,40},{-90,40}},
                                       color={0,0,127}));
        connect(getInputs.V_flow_in, V_flow_in) annotation (Line(points={{-42,4.2},{-70,
                4.2},{-70,40},{-90,40}},
                                       color={0,0,127}));

        annotation (Icon(coordinateSystem(preserveAspectRatio=true,
                            extent={{-40,-100},{40,100}}),
                         graphics={Text(
                            extent={{-100,-70},{100,-110}},
                            lineColor={0,0,0},
                            textString="%boundaryType")}),
            Documentation(info="<html>
        <p>
        Boundary models determine the conditions at the boundary of a system.
        <br>
        The user is able to determine:
        </p>
        <ul>
          <li> enthalpy (h) or temperature (T)</li>
          <li> mass fraction (xi), mole fraction (x) or relative humidity (phi) *</li>
          <li> pressure (p), mass flow rate (m_flow) or volume flow rate (V_flow)</li>
        </ul>
        <p>
        All values for the boundary can be given both as fixed parameter or with a real-input connector.
        Boundaries have an egoistic notation of flow variables. 
        A negative mass flow rate leaves the boundary and vice versa.
        h, T, xi, phi and x are stream variables.
        Stream variables are convectively transported with the mass flow.
        If the mass flow is negative (leaving the boundary), the stream information of the boundary can be measured at the outlet.
        If the mass flow is positive (entering the boundary), the stream information of the connected component can be measured.
        <br>
        <br>
        If enthalpy (h) is given inside/for the boundary, it is only meaningful to use mass fraction (xi) or mole fraction (x) additionally.
        The combination of enthalpy (h) and relative humidity (phi) is not reasonable.
        <br>
        The temperature (T) can be given also in combination with the relative humidity (phi).
        If a gas mixture with more than two components including a condensing component (e.g. &quot;DryAir|...|Water&quot;) will be used, it is also possible to define the relative humidity (phi).
        But the proportions of the not condensable gas components of the mixture will be calculated with the given mass fraction (xi) inside/for the boundary.
        In this case temperature (T), relative humidity (phi) and the mixing ratio array less one value for the condensing component will be used.
        <br>
        <br>
        *
        The user specifies the mass or molar fraction using the corresponding mixing ratio.
        The mixing ratio is an array with the ratio of all mixture components.
        Its sum might exceed the value one.
        Out of the mixing ratio, the mass or molar fraction is calculated by normalizing.
        In case a condensing component is present, the user can additionally specify the relative humidity.
        </p>
        <hr>
        </html>"));
      end Boundary;

      package BaseClasses
      import TIL;
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialGasBoundary "Partial gas boundary"

        /*********************** SIM ***********************************/

          parameter TILMedia.GasTypes.BaseGas           gasType = sim.gasType1
            "Gas type" annotation (Dialog(tab="SIM",group="SIM"),choices(
            choice=sim.gasType1 "Gas 1 as defined in SIM",
            choice=sim.gasType2 "Gas 2 as defined in SIM",
            choice=sim.gasType3 "Gas 3 as defined in SIM"));
        protected
          outer SystemInformationManager sim "System information manager";

        /******************** Connector *****************************/

        public
          TIL.Connectors.GasPort port(final gasType = gasType) "Port"
            annotation (Placement(transformation(extent={{-10,10},{10,30}}, rotation=0),
                iconTransformation(extent={{-10,-10},{10,10}})));

          Modelica.Blocks.Interfaces.RealInput T_in if use_temperatureInput
            "Prescribed boundary temperature [K]"
            annotation (Placement(transformation(extent={{-100,-50},{-80,-30}},
                  rotation=0), iconTransformation(extent={{-50,-70},{-30,-50}})));
          Modelica.Blocks.Interfaces.RealInput h_in if use_specificEnthalpyInput
            "Prescribed boundary specific enthalpy [J/kg]"
            annotation (Placement(transformation(extent={{-100,-50},{-80,-30}},
                  rotation=0), iconTransformation(extent={{-50,-70},{-30,-50}})));
          Modelica.Blocks.Interfaces.RealInput phi_in if use_relativeHumidityInput
            "Prescribed boundary relative humidity [%]"
            annotation (Placement(transformation(extent={{-100,-10},{-80,10}}, rotation=
                   0), iconTransformation(extent={{-50,-30},{-30,-10}})));
          Modelica.Blocks.Interfaces.RealInput[ gasType.nc] mixingRatio_in if use_mixingRatioInput
            "Prescribed boundary humidity concentration [1]"
            annotation (Placement(transformation(extent={{-100,-10},{-80,10}}, rotation=
                   0), iconTransformation(extent={{-50,-30},{-30,-10}})));
          Modelica.Blocks.Interfaces.RealInput[ gasType.nc] molarMixingRatio_in if use_molarMixingRatioInput
            "Prescribed boundary humidity concentration [1]"
            annotation (Placement(transformation(extent={{-100,-10},{-80,10}}, rotation=
                   0), iconTransformation(extent={{-50,-30},{-30,-10}})));

        /******************** Gas Objects ***************************/
        protected
          TILMedia.Gas_ph gas_ph(final p=p, final h=h, final xi=xi, final gasType = gasType,
            phi(start=0.2), computeTransportProperties=false) "Gas object"
            annotation (Placement(transformation(extent={{60,20},{80,40}}, rotation=0)));

          TILMedia.Gas_pT gas_pT(final p=p, final T=T, final xi=xi, final gasType = gasType,
            phi(start=0.2), computeTransportProperties=false) "Gas object"
            annotation (Placement(transformation(extent={{40,20},{60,40}}, rotation=0)));

          TILMedia.Gas_ph gas_inStream(
            final p=port.p,
            final h=inStream(port.h_outflow),
            final xi=inStream(port.xi_outflow),
            final gasType=gasType,
            computeTransportProperties=false)
            "Gas properties in case of instreaming flow"
            annotation (Placement(transformation(extent={{60,-40},{80,-20}}, rotation=0)));

        /******************** Stream Variables ********************/
        public
          parameter String streamVariablesInputType="T"
            "Constrained variable, if fluid flows out of boundary"
            annotation(Dialog(group="Input Type", tab="Gas Property Inputs: T, h, phi, xi, x"),
            choices(choice="T", choice="h"));

           parameter String streamVariablesInputTypeConcentration="xi"
            "Stream variables input type for concentration information"
            annotation(Dialog(group="Input Type", tab="Gas Property Inputs: T, h, phi, xi, x"),
            choices(choice="phi", choice="xi", choice="x"));
        protected
          Real fractionSum;

        public
          parameter Boolean use_temperatureInput=false "= true, if T defined by input"
                                            annotation(Dialog(enable=(streamVariablesInputType=="T"), tab = "Gas Property Inputs: T, h, phi, xi, x", group = "T - Temperature"));
          parameter SI.Temperature TFixed=298.15 "Fixed value for temperature"
            annotation(Dialog(enable=(not use_temperatureInput and (streamVariablesInputType=="T")), tab = "Gas Property Inputs: T, h, phi, xi, x", group = "T - Temperature"));

          parameter Boolean use_specificEnthalpyInput=false
            "= true if h defined by input"
            annotation(Dialog(enable=(streamVariablesInputType=="h"),
            group="h - Specific Enthalpy", tab = "Gas Property Inputs: T, h, phi, xi, x"));
          parameter SI.SpecificEnthalpy hFixed=350.0e3 "Fixed value for h"
            annotation(Dialog(enable=(not use_specificEnthalpyInput and (streamVariablesInputType=="h")),
            group="h - Specific Enthalpy", tab = "Gas Property Inputs: T, h, phi, xi, x"));

          parameter Boolean use_relativeHumidityInput=false
            "= true if phi defined by input"
            annotation(Dialog(enable=(streamVariablesInputTypeConcentration=="phi"),
            tab="Gas Property Inputs: T, h, phi, xi, x", group="phi - Relative Humidity (use only for MoistAir or if condensing component in gas mixture)"));

          parameter TILMedia.Internals.Units.RelativeHumidity
            phiFixed = 60.0 "Fixed value for gas relative humidity (0%-100%)"
            annotation(Dialog(enable=(not use_relativeHumidityInput and (streamVariablesInputTypeConcentration=="phi")),
             tab="Gas Property Inputs: T, h, phi, xi, x", group="phi - Relative Humidity (use only for MoistAir or if condensing component in gas mixture)"));

          parameter Boolean use_mixingRatioInput=false "= true if xi defined by input"
            annotation(Dialog(enable=streamVariablesInputTypeConcentration=="xi" or (gasType.nc>2 and streamVariablesInputTypeConcentration=="phi"),
             tab="Gas Property Inputs: T, h, phi, xi, x", group="xi - Mass Fraction (potentially used in combination with phi, if gas components > 2)"));

          parameter Real[gasType.nc] mixingRatioFixed=gasType.defaultMixingRatio
            "Fixed array for mass fraction of gas components e.g. {2, 10, 0.3}"
            annotation(Dialog(enable=(not use_mixingRatioInput and (streamVariablesInputTypeConcentration=="xi" or (gasType.nc>2 and streamVariablesInputTypeConcentration=="phi"))),
            tab="Gas Property Inputs: T, h, phi, xi, x", group="xi - Mass Fraction (potentially used in combination with phi, if gas components > 2)"));
          parameter Boolean use_molarMixingRatioInput=false
            "= true if x defined by input"
            annotation(Dialog(enable=(streamVariablesInputTypeConcentration=="x"),
            tab="Gas Property Inputs: T, h, phi, xi, x", group="x - Mole Fraction"));
          parameter Real[ gasType.nc] molarMixingRatioFixed= ones(gasType.nc)
            "Fixed array for mole fraction of gas components e.g. {2, 10, 0.3}"
            annotation(Dialog(enable=(not use_molarMixingRatioInput and streamVariablesInputTypeConcentration=="x"),
            tab="Gas Property Inputs: T, h, phi, xi, x", group="x - Mole Fraction"));

        /******************** Protected Variables ********************/

        protected
          SI.MassFlowRate m_flow "Mass flow rate";
          SI.Density d "Density";
          SI.SpecificEnthalpy h "Specific enthalpy";
          SI.AbsolutePressure p "Pressure";
          SI.Temperature T "Temperature";
          SI.MassFraction[gasType.nc-1] xi "Mass fraction";

          Modelica.Blocks.Sources.Constant T_in_(k=TFixed) if not use_temperatureInput;
          Modelica.Blocks.Sources.Constant h_in_(k=hFixed) if not use_specificEnthalpyInput;
          Modelica.Blocks.Sources.Constant[gasType.nc] mixingRatio_in_(k=mixingRatioFixed) if not use_mixingRatioInput;
          Modelica.Blocks.Sources.Constant[gasType.nc] molarMixingRatio_in_(k=molarMixingRatioFixed) if not use_molarMixingRatioInput;
          Modelica.Blocks.Sources.Constant phi_in_(k=phiFixed) if not use_relativeHumidityInput;

          TIL.GasComponents.Boundaries.Internals.GetInputs getInputs(final gasType=gasType)
            annotation (Placement(transformation(extent={{-40,-10},{-20,10}}, rotation=
                    0)));

          replaceable record SummaryClass = Summary;

        record Summary
            extends TIL.Internals.ClassTypes.Record;
          SI.AbsolutePressure p "Pressure at port";
          SI.Temperature T "Temperature at port";
          SI.Temperature T_outflow "Outflowing temperature";
          SI.SpecificEnthalpy h "Specific enthalpy at port";
          TILMedia.Internals.Units.RelativeHumidity phi "Relative humidity at port";
          SI.MassFraction[nc-1] xi
              "Mass fraction of independent gas components at port";
          SI.MassFlowRate m_flow "Mass flow rate at port";
          SI.VolumeFlowRate V_flow "Volume flow rate at port";
          parameter Integer nc "Number of gas components";
        end Summary;

        public
          SummaryClass summary(p=p,
                T = if noEvent(port.m_flow>0) then gas_inStream.T else T,
                T_outflow = T,
                h = if noEvent(port.m_flow>0) then gas_inStream.h else h,
                phi = if not
                            (gasType.condensingIndex>0) then 0 elseif
                            (noEvent(port.m_flow>0) and gasType.condensingIndex>0) then
                  gas_inStream.phi else gas_ph.phi,
                xi = if noEvent(port.m_flow>0) then gas_inStream.xi else xi,
                m_flow = port.m_flow,
                V_flow = if noEvent(port.m_flow>0) then port.m_flow/gas_inStream.d else port.m_flow/d,
                nc = gasType.nc);

        equation
          p = port.p;
          m_flow = port.m_flow;
          port.h_outflow = h;
          port.xi_outflow = xi;

          if (streamVariablesInputTypeConcentration == "xi") then
            xi = getInputs.mixingRatio_in[1:end-1]/sum(getInputs.mixingRatio_in);
            fractionSum = sum(getInputs.mixingRatio_in);
          elseif  (streamVariablesInputTypeConcentration == "phi") and (gasType.nc>1) then
            assert(gasType.condensingIndex>0,"phi can only be set if a vapour-gas mixture is used (if a component can condense)");
            fractionSum = sum(getInputs.mixingRatio_in) - getInputs.mixingRatio_in[max(gasType.condensingIndex,1)];
            gas_pT.phi = getInputs.phi_in;
            //if phi was given use only nc-2 mass fractions and xi_dryGas.
            for i in 1:(gasType.nc-2) loop
              if i<gasType.condensingIndex then
                gas_pT.xi_dryGas[i] = getInputs.mixingRatio_in[i]/fractionSum;
              else
                gas_pT.xi_dryGas[i] = getInputs.mixingRatio_in[i+1]/fractionSum;
              end if;
            end for;
          else
            gas_pT.x = getInputs.molarMixingRatio_in[1:end-1]/fractionSum;
            fractionSum=sum(getInputs.molarMixingRatio_in);
          end if;

          if (streamVariablesInputType=="h") then
            T = gas_ph.T;
            d = gas_ph.d;
            h = getInputs.h_in;
          else
            h = gas_pT.h;
            d = gas_ph.d;
            T = getInputs.T_in;
          end if;

          connect(getInputs.T_in, T_in_.y);
          connect(getInputs.h_in, h_in_.y);
          connect(getInputs.mixingRatio_in, mixingRatio_in_.y);
          connect(getInputs.molarMixingRatio_in, molarMixingRatio_in_.y);
          connect(getInputs.phi_in, phi_in_.y);

          connect(T_in, getInputs.T_in) annotation (Line(points={{-90,-40},{-60,-40},{
                  -60,-10},{-42,-10}},          color={0,0,127}));
          connect(getInputs.h_in, h_in) annotation (Line(points={{-42,-7.2},{-60,-7.2},{
                  -60,-40},{-90,-40}},
                                   color={0,0,127}));
          connect(getInputs.mixingRatio_in, mixingRatio_in) annotation (Line(points={{-42,-1.6},{-60,-1.6},
                  {-60,0},{-90,0}},color={0,0,127}));
          connect(getInputs.molarMixingRatio_in, molarMixingRatio_in) annotation (Line(points={{-42,-4.4},{-60,-4.4},{
                  -60,0},{-90,0}}, color={0,0,127}));
          connect(phi_in, getInputs.phi_in) annotation (Line(points={{-90,0},{-60,0},{
                  -60,1.2},{-42,1.2}},                                                color={0,0,127}));

          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-100},
                    {40,100}}),
                           graphics={ Rectangle(
                  extent={{2,50},{-2,-50}},
                  lineColor={255,153,0},
                  fillColor={255,153,0},
                  fillPattern=FillPattern.Solid)}),
                                  Diagram(graphics));
        end PartialGasBoundary;
      end BaseClasses;

      package Internals
       extends TIL.Internals.ClassTypes.InternalPackage;

        model GetInputs "Get enabled inputs and parameters of disabled inputs"
          extends Modelica.Blocks.Interfaces.BlockIcon;
          parameter TILMedia.GasTypes.BaseGas           gasType;
          Modelica.Blocks.Interfaces.RealInput T_in "Prescribed boundary temperature"
            annotation (Placement(transformation(extent={{-140,-120},{-100,-80}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput m_flow_in
            "Prescribed boundary mass flow rate"
            annotation (Placement(transformation(extent={{-140,52},{-100,92}}, rotation=
                   0)));
          Modelica.Blocks.Interfaces.RealInput V_flow_in
            "Prescribed boundary volume flow rate"
            annotation (Placement(transformation(extent={{-140,22},{-100,62}}, rotation=
                   0)));
          Modelica.Blocks.Interfaces.RealInput p_in "Prescribed boundary pressure"
            annotation (Placement(transformation(extent={{-140,80},{-100,120}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput h_in
            "Prescribed boundary specific enthalpy"
            annotation (Placement(transformation(extent={{-140,-92},{-100,-52}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput phi_in
            "Prescribed boundary relative humidity"
            annotation (Placement(transformation(extent={{-140,-8},{-100,32}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput[ gasType.nc] mixingRatio_in
            "Prescribed boundary humidity concentration"
            annotation (Placement(transformation(extent={{-140,-36},{-100,4}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput[ gasType.nc] molarMixingRatio_in
            "Prescribed boundary humidity concentration"
            annotation (Placement(transformation(extent={{-140,-64},{-100,-24}},
                  rotation=0)));
          annotation (Diagram(graphics));
        end GetInputs;
      annotation(classOrder={"BoundaryType","PartialMoistAirBoundary","GetInputs","FlowAndHumidityDimensionSwitch","*"});
      end Internals;
     annotation(classOrder={"*","BaseClasses","Internals","Testers"});
    end Boundaries;

    package Fans "Fans"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model SimpleFan "Simple fan model"
       extends TIL.GasComponents.Fans.BaseClasses.PartialFan(orientation="symmetric",
            summary(P_loss = P_drive-P_hyd), dp(final start=100));

        SI.Power P_drive "Drive power";

      /********************** Boundary conditions ***********************/

        parameter TIL.Internals.PresetVariableType presetVariableType = "V_flow"
          "Specifies which variable is preset"
          annotation(Dialog(group="General Settings"));

        parameter Boolean use_dpInput=false "= true, if dp defined by input"
          annotation(Dialog(enable=(presetVariableType=="dp"), group="Pressure Increase"));
        parameter SI.Pressure dpFixed = 0.1e5 "Fixed value for pressure increase"
          annotation(Dialog(enable=(not use_dpInput and presetVariableType=="dp"),group="Pressure Increase"));
        parameter Boolean use_massFlowRateInput=false
          "= true, if m_flow defined by input"
          annotation(Dialog(enable=(presetVariableType=="m_flow"), group="Mass Flow Rate"));
        parameter SI.MassFlowRate m_flowFixed=0.5
          "Fixed value for gas mass flow rate"
          annotation(Dialog(enable=(not use_massFlowRateInput and presetVariableType=="m_flow"),group="Mass Flow Rate"));
        parameter Boolean use_volumeFlowRateInput=false
          "= true, if V_flow defined by input"
          annotation(Dialog(enable=(presetVariableType=="V_flow"), group="Volume Flow Rate"));
        parameter SI.VolumeFlowRate V_flowFixed=0.5e-3
          "Fixed value for gas volume flow rate"
          annotation(Dialog(enable=(not use_volumeFlowRateInput and presetVariableType=="V_flow"),group="Volume Flow Rate"));
        parameter Boolean isenthalpicProcess=false "= true, if h_out = h_in"
          annotation(Dialog(tab="Advanced",group="Energy balance"));

      /*********************** Inputs *******************************/

        Modelica.Blocks.Interfaces.RealInput dp_in if
          use_dpInput "Prescribed pressure increase [Pa]"
          annotation (Placement(transformation(extent={{-100,50},{-80,70}}, rotation=
                  0), iconTransformation(extent={{-110,50},{-90,70}})));
        Modelica.Blocks.Interfaces.RealInput m_flow_in if
                                                        use_massFlowRateInput
          "Prescribed mass flow rate [kg/s]"
          annotation (Placement(transformation(extent={{-100,-70},{-80,-50}},
                rotation=0), iconTransformation(extent={{-110,-70},{-90,-50}})));
        Modelica.Blocks.Interfaces.RealInput V_flow_in if use_volumeFlowRateInput
          "Prescribed volume flow rate [m^3/s]"
          annotation (Placement(transformation(extent={{-100,-10},{-80,10}}, rotation=
                 0), iconTransformation(extent={{-110,-10},{-90,10}})));

      protected
        Modelica.Blocks.Sources.Constant dp_in_(k=0) if not use_dpInput;
        Modelica.Blocks.Sources.Constant m_flow_in_(k=0) if not use_massFlowRateInput;
        Modelica.Blocks.Sources.Constant V_flow_in_(k=0) if not use_volumeFlowRateInput;

      /************************* Additional parameters ***************/

      public
        parameter SI.Efficiency eta=0.4
          "Fan efficiency (used to calculate Shaft Power, which is entirely added to Fluid's Energy Balance)"
          annotation(Dialog(group="General Settings"));
        parameter SI.Efficiency etaDrive=1.0
          "Drive efficiency (used to calculate Drive Power; does not affect Fluid's Energy Balance)"
          annotation(Dialog(enable=includeDrive, group="General Settings"));
      protected
        TIL.Internals.GetInputsHydraulic getInputs
                                      annotation (Placement(transformation(extent={{
                  -56,-10},{-36,10}}, rotation=0)));

      equation
        volumeFlowRate = portA.m_flow / gas.d;

      //____________________ Boundary equations _________________

        if presetVariableType == "dp" then
          if use_dpInput then
            dp = getInputs.dp_in;
          else
            dp = dpFixed;
          end if;
        elseif presetVariableType == "m_flow" then
          if use_massFlowRateInput then
            portA.m_flow = getInputs.m_flow_in;
          else
            portA.m_flow = m_flowFixed;
          end if;
        else
          if use_volumeFlowRateInput then
            volumeFlowRate = getInputs.V_flow_in;
          else
            volumeFlowRate = V_flowFixed;
          end if;
        end if;

      //____________________ Calculate Power _____________________

        P_hyd = dp*volumeFlowRate;
        P_shaft = 1/eta*P_hyd;
        P_drive = 1/eta*1/etaDrive*P_hyd;

      //____________________ Balance equations ___________________

      // Energy balance
        if isenthalpicProcess then
          portB.h_outflow = inStream(portA.h_outflow);
        else
          portB.h_outflow = inStream(portA.h_outflow) + P_hyd/eta/portA.m_flow;
        end if;
        portA.h_outflow = inStream(portB.h_outflow);

      // Mass and Momentum balance see partial class

      //_____________________ Connections ________________________
        connect(dp_in_.y, getInputs.dp_in);
        connect(V_flow_in_.y, getInputs.V_flow_in);
        connect(m_flow_in_.y, getInputs.m_flow_in);
        connect(dp_in, getInputs.dp_in) annotation (Line(points={{-90,60},{-76,60},{
                -76,6},{-58,6}}, color={0,0,127}));
        connect(V_flow_in, getInputs.V_flow_in)
          annotation (Line(points={{-90,0},{-58,0}}, color={0,0,127}));
        connect(m_flow_in, getInputs.m_flow_in) annotation (Line(points={{-90,-60},{
                -76,-60},{-76,-6},{-58,-6}}, color={0,0,127}));

      annotation (         Icon(
            coordinateSystem(preserveAspectRatio=true, extent={{-80,-80},{80,80}}),
              Polygon(
                points={{-100,10},{-80,0},{-100,-10},{-100,10}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
            graphics),
            Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>differential states:</td><td>-</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure increase</td>
        </tr>
        </table>
        <p>
        The Simple Fan Model defines a pressure increase, a mass flow rate or a volume flow rate. 
        One of the variables can be a fixed value or set by a changing input from outside.
        Also a fan efficiency is needed to calculate the shaft power, which is entirely added to fluid's energy balance.
        The drive efficiency is used to calculate the drive power, which does not affect fluid's energy balance.
        </p>
        <hr>
        </html>"));
      end SimpleFan;

      package BaseClasses
      import TIL;
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialFan

        /*********************** SIM ***********************************/

          parameter TILMedia.GasTypes.BaseGas           gasType = sim.gasType1
            "Gas type" annotation (Dialog(tab="SIM",group="SIM"),choices(
            choice=sim.gasType1 "Gas 1 as defined in SIM",
            choice=sim.gasType2 "Gas 2 as defined in SIM",
            choice=sim.gasType3 "Gas 3 as defined in SIM"));
        protected
          outer SystemInformationManager sim "System information manager";

        /******************** Connectors *****************************/

        public
          TIL.Connectors.GasPort portA(m_flow(final start = m_flowStart),
            final gasType = gasType) "Fluid inlet Port"
            annotation (Placement(transformation(extent={{-10,-90},{10,-70}}, rotation=0)));

          TIL.Connectors.GasPort portB(m_flow(final start = -m_flowStart),
            final gasType = gasType) "Fluid outlet Port"
            annotation (Placement(transformation(extent={{-10,70},{10,90}}, rotation=0)));

        /****************** Gas object *************************/

          parameter TIL.Internals.gasPressureOrientation orientation= "A"
            "Pressure orientation used to calculate gas properties and volume flow rate"
            annotation(Dialog(tab="Advanced", group="Gas"));
        protected
          TILMedia.Gas_ph   gas(p=p,
            h=inStream(portA.h_outflow),
            xi=inStream(portA.xi_outflow),
            final gasType = gasType,
            computeTransportProperties=false) "Gas in fan"
            annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));
        /****************** Start Values ********************/

         parameter Modelica.SIunits.MassFlowRate m_flowStart = 1e-3
            "Mass flow rate at start" annotation(Dialog(tab="Start Values"));
        /****************** Additional variables **************/
          SI.Pressure p "pressure used for gas property calculation";
        public
          SI.Pressure dp "pressure increase";
          SI.VolumeFlowRate volumeFlowRate;
          SI.Power P_hyd "Hydraulic power";
          SI.Power P_shaft "Shaft power";

          /*******************Summary**********************/

          parameter Boolean includeDefaultSummary = true
            "Include summary record in model results"
         annotation(Dialog(tab="Advanced", group="Summary"));

        protected
          record Summary
            extends TIL.Internals.ClassTypes.Record;

            SI.Pressure p_A "Pressure at port A";
            SI.Pressure p_B "Pressure at port B";
            SI.Temperature T "Temperature";
            SI.Temp_C T_degC "Temperature";
            SI.SpecificEnthalpy h "Specific enthalpy";
            SI.Density d "Density";
            SI.Pressure dp "Total pressure increase";
            SI.MassFlowRate m_flow_A "Mass flow rate at port A";
            SI.MassFlowRate m_flow_B "Mass flow rate at port B";

            SI.VolumeFlowRate V_flow "Volume flow rate";
            SI.Power P_shaft "Shaft Power";
            SI.Power P_loss "Total loss power";

          end Summary;

          replaceable record SummaryClass = Summary;

        public
          SummaryClass summary(
            p_A = portA.p,
            p_B = portB.p,
            T = gas.T,
            T_degC = gas.T - 273.15,
            h = gas.h,
            d = gas.d,
            dp = portB.p - portA.p,
            m_flow_A = portA.m_flow,
            m_flow_B = portB.m_flow,
            V_flow = volumeFlowRate,
            P_shaft = P_shaft) if includeDefaultSummary
            annotation (Placement(transformation(extent={{40,-80},{60,-60}}, rotation=0)));

        equation
          if orientation =="A" then
            p=portA.p;
          elseif orientation == "B" then
            p=portB.p;
          else
            p=(portA.p + portB.p)/2;
          end if;

        // Momentum balance
          dp = portB.p - portA.p;

        // Mass balance
           portA.m_flow + portB.m_flow = 0.0;
        // Component mass balance
           portB.xi_outflow = inStream(portA.xi_outflow);
           portA.xi_outflow = inStream(portB.xi_outflow);

          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-80,
                    -80},{80,80}})), Icon(coordinateSystem(preserveAspectRatio=true,
                  extent={{-80,-80},{80,80}}), graphics={
                        Bitmap(
                  extent={{-80,-80},{80,80}},
                  imageSource=
                      "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",
                  fileName="modelica://TIL/Images/FanUni.png")}));
        end PartialFan;
      end BaseClasses;

      package Internals
        import TIL;
        extends TIL.Internals.ClassTypes.InternalPackage;

        function calcCoefficients
         input Real V_flow_1, dp_1, V_flow_2, deltaV_flow;
         output TIL.GasComponents.Fans.Internals.Coefficients coef;

        protected
          Real den;
          Real V_flow_max;
          Real V_flow_3;

        algorithm
          if deltaV_flow > 0 then
              V_flow_3  := V_flow_2 + deltaV_flow;
              den := (V_flow_3-V_flow_1)*(V_flow_2-V_flow_1)*(V_flow_2-V_flow_1);
              coef.c2  := (dp_1*(V_flow_2-V_flow_3))/den;
              coef.c1  := (2*V_flow_1*(V_flow_3-V_flow_1)*(dp_1)+dp_1*(V_flow_1*V_flow_1-V_flow_2*V_flow_2))/den;
              coef.dp_0  := (V_flow_1*(V_flow_1*(V_flow_2*dp_1)-2*V_flow_3*V_flow_2*dp_1) + dp_1*V_flow_3*V_flow_2*V_flow_2)/den;

              V_flow_max := -coef.c1/coef.c2/2;
              coef.V_flow_transition := (V_flow_1 + V_flow_max)/2;
          else
              V_flow_3  := V_flow_2;
              den := 0;
              coef.c2  := 0;
              coef.c1  := dp_1/(V_flow_1-V_flow_2);
              coef.dp_0  := -coef.c1*V_flow_2;

              V_flow_max := V_flow_1;
              coef.V_flow_transition := V_flow_1;
          end if;

        end calcCoefficients;

        record Coefficients
          Real dp_0, c1, c2;
          Real V_flow_transition;
        end Coefficients;

        function semiSquareFunction
         input Real V_flow;
         input TIL.GasComponents.Fans.Internals.Coefficients coef;
         output Real dp;

        protected
          Real dp_transition, ddpdVflow_transition;

        algorithm
          if coef.c2 <> 0 then
              dp_transition := coef.V_flow_transition*(coef.c2*coef.V_flow_transition + coef.c1) + coef.dp_0;
              ddpdVflow_transition := 2*coef.c2*coef.V_flow_transition + coef.c1;

              if V_flow < coef.V_flow_transition then
                dp := dp_transition + (V_flow - coef.V_flow_transition)*ddpdVflow_transition;
              else
                dp := V_flow*(coef.c2*V_flow + coef.c1) + coef.dp_0;
              end if;
          else
              dp_transition := 0;
              ddpdVflow_transition := 0;

              dp := coef.c1*V_flow + coef.dp_0;
          end if;

          annotation(inverse(V_flow = semiSquareFunction_inverse(dp,coef)));
        end semiSquareFunction;

        function semiSquareFunction_inverse
         input Real dp;
         input TIL.GasComponents.Fans.Internals.Coefficients coef;
         output Real V_flow;
        protected
          Real dp_transition, ddpdVflow_transition;

        algorithm
          if coef.c2 <> 0 then
              dp_transition := coef.V_flow_transition*(coef.c2*coef.V_flow_transition + coef.c1) + coef.dp_0;
              ddpdVflow_transition := 2*coef.c2*coef.V_flow_transition + coef.c1;

              if dp > dp_transition then
                //dp := dp_transition + (V_flow - coef.V_flow_transition)*ddpdVflow_transition;
                V_flow := (dp-dp_transition)/ddpdVflow_transition + coef.V_flow_transition;
              else
                //dp := V_flow*(coef.c2*V_flow + coef.c1) + coef.dp_0;
                // 0 = V_flow^2 + coef.c1/coef.c2*V_flow + (coef.dp_0 - dp)/coef.c2;
                V_flow := -coef.c1/coef.c2/2 + sqrt( (coef.c1/coef.c2/2)^2 - (coef.dp_0-dp)/coef.c2);
              end if;
          else
              dp_transition := 0;
              ddpdVflow_transition := 0;
              //dp := coef.c1*V_flow + coef.dp_0;
              V_flow :=(dp - coef.dp_0)/coef.c1;
          end if;

        end semiSquareFunction_inverse;
      end Internals;
    annotation (classOrder={"*","BaseClasses","Internals","Testers"});
    end Fans;

    package HydraulicResistors
    extends Internals.ClassTypes.ComponentPackage;

      model HydraulicResistor "Hydraulic resistor"

      /*********************** SIM ***********************************/

        parameter TILMedia.GasTypes.BaseGas           gasType = sim.gasType1
          "Gas type" annotation (Dialog(tab="SIM",group="SIM"),choices(
          choice=sim.gasType1 "Gas 1 as defined in SIM",
          choice=sim.gasType2 "Gas 2 as defined in SIM",
          choice=sim.gasType3 "Gas 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";

      /******************** Connectors *****************************/

      public
        Connectors.GasPort portA(
          p(final start=pStart),
          h_outflow(final start=hStart),
          xi_outflow(final start=xiStart),
          m_flow(final start=m_flowStart),
          final gasType = gasType)
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}}, rotation=
                  0), iconTransformation(extent={{-70,-10},{-50,10}})));
        Connectors.GasPort portB(
          p(final start=pStart),
          h_outflow(final start=hStart),
          xi_outflow(final start=xiStart),
          final gasType = gasType)
          annotation (Placement(transformation(extent={{50,-10},{70,10}}, rotation=0),
              iconTransformation(extent={{50,-10},{70,10}})));

       /****************** General parameters *******************/

        parameter String inputChoice = "zeta" "|Input|" annotation(choices(choice = "zeta", choice = "dp"));
        parameter Real zeta_fixed = 1 "pressure loss coefficient"
                                        annotation(Dialog(group="Input", enable = inputChoice == "zeta"));
        parameter Modelica.SIunits.Pressure dp_nominal = 1 "nominal pressure drop"
                                    annotation(Dialog(group="Input", enable = inputChoice == "dp"));
        parameter Modelica.SIunits.VolumeFlowRate Vdot_nominal = 1
          "nominal volume flow rate"     annotation(Dialog(group="Input", enable = inputChoice == "dp"));

        parameter Modelica.SIunits.Diameter hydraulicDiameter = 0.02
          "represented hydraulic diameter"   annotation(Dialog(group="Geometry", enable = inputChoice == "zeta"));

        output Modelica.SIunits.Pressure pressureDrop "calculated pressure drop";

        parameter Modelica.SIunits.Pressure p_nominal = 1.013e5  annotation(Dialog(group="Nominal state", enable = inputChoice == "dp"));
        parameter Modelica.SIunits.Temperature T_nominal = 298.15  annotation(Dialog(group="Nominal state", enable = inputChoice == "dp"));
        //parameter Modelica.SIunits.MassFraction xi_nominal = 0.001  annotation(Dialog(group="Nominal state", enable = inputChoice == "dp"));
        parameter Real[gasType.nc] mixingRatio_nominal = gasType.defaultMixingRatio
          "Initial value for mixing ratio" annotation(Dialog(group="Nominal state", enable = inputChoice == "dp"));

        parameter Modelica.SIunits.Pressure dpSmooth=10
          "below this value, root function is approximated linearly" annotation(Dialog(tab="Advanced",group="Smoothing Function"));

        final parameter Boolean generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal  annotation(Evaluate=true);

       /****************** Start values *******************/

        parameter Modelica.SIunits.Pressure pStart = p_nominal annotation(Dialog(tab="Start Values"));
        parameter Modelica.SIunits.SpecificEnthalpy hStart = TILMedia.GasFunctions.specificEnthalpy_pTxi(gasType, p_nominal, T_nominal, mixingRatio_nominal[1:end-1]/sum(mixingRatio_nominal)) annotation(Dialog(tab="Start Values"));
        parameter Modelica.SIunits.MassFraction xiStart[gasType.nc - 1] = mixingRatio_nominal[1:end-1]/sum(mixingRatio_nominal) annotation(Dialog(tab="Start Values"));
        parameter Modelica.SIunits.MassFlowRate m_flowStart = 0.01 annotation(Dialog(tab="Start Values"));
      protected
        TILMedia.Gas_ph gasA(
          p=portA.p, h=inStream(portA.h_outflow),
          xi = inStream(portA.xi_outflow),
          final gasType = gasType,
          computeTransportProperties=false)
          annotation (Placement(transformation(extent={{-100,20},{-80,40}}, rotation=0)));
        TILMedia.Gas_ph gasB(
          p=portB.p,
          h=inStream(portB.h_outflow),
          xi = inStream(portB.xi_outflow),
          final gasType = gasType,
          computeTransportProperties=false)
          annotation (Placement(transformation(extent={{80,20},{100,40}}, rotation=0)));
        Modelica.SIunits.Area A;
        Modelica.SIunits.Velocity w;
        Modelica.SIunits.Density d_in;
        Modelica.SIunits.Density density_nominal = TILMedia.GasFunctions.density_pTxi(gasType, p_nominal, T_nominal, mixingRatio_nominal[1:end-1]/sum(mixingRatio_nominal));

      public
        Modelica.SIunits.VolumeFlowRate Vdot;

      equation
        A=hydraulicDiameter*hydraulicDiameter*Modelica.Constants.pi/4;
        Vdot=portA.m_flow/d_in;
        w=Vdot/A;

        portB.h_outflow = inStream(portA.h_outflow);
        portA.h_outflow = inStream(portB.h_outflow);

        portB.xi_outflow = inStream(portA.xi_outflow);
        portA.xi_outflow = inStream(portB.xi_outflow);

        portA.m_flow + portB.m_flow = 0;

        pressureDrop = portA.p - portB.p;

        if ((generateEventsAtFlowReversal and portA.p > portB.p)
          or (not generateEventsAtFlowReversal and noEvent(portA.p > portB.p))) then
          d_in = gasA.d;
        else
          d_in = gasB.d;
        end if;
        if inputChoice == "zeta" then
          Vdot = A* sqrt(1/(zeta_fixed * d_in/2))*TIL.Utilities.Numerics.squareRootFunction(pressureDrop, dpSmooth);
        else
          Vdot = sqrt(1/(dp_nominal * d_in/density_nominal/Vdot_nominal/Vdot_nominal))*TIL.Utilities.Numerics.squareRootFunction(pressureDrop, dpSmooth);
        end if;

      //    preferedView="info",

      /*      <p>
      HydraulicResistor is a simple pressure drop model.
      </p>
      <p>
      For a given dp:
      <br>&Delta;p = &Delta;p_0 * &rho;/&rho;_0 * (Vdot/Vdot_0)        &sup2;
      <br>Vdot =         &plusmn; sqrt(|&Delta;p|/&Delta;p_0 * &rho;_0/&rho; Vdot_0&sup2;)
      </p>
      <p>
      For a given zeta:
      <br>&Delta;p = &zeta; * &rho;/2 * w&sup2;
      <br>Vdot = A * w
      <br>Vdot = &plusmn; A * sqrt(|&Delta;p|*2 / (&Delta;p * &zeta;))
      </p>*/
        annotation (
          Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>differential states:</td><td>-</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure drop</td>
        </tr>
        </table>
      <p>
      HydraulicResistor is a quadratic pressure drop model.
      It's possible to enter a zeta value or a nominal pressure drop at a nominal volume flow rate.
      In order to avoid a zero slope in the origin, which generally leads to numerical problems, the quadratic function is modified near the origin.
      A 3rd order polynom with smooth links to the quadratic function and an inflection point at the origin is used for.
      </p>
      </html>"),
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-60,-20},{60,20}},
              initialScale=0.1), graphics={Bitmap(extent={{-60,-20},{60,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAHgAAAAoCAIAAAC6iKlyAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAlBJREFUeNrsmuGRAUEQhbnynwxsBmTgREAGiIAMCEEGZEAGiAARWBFwEey92qlqc7MzU3dX3Pbxvh9bs2N2a71529M9VCqEEEK0UrVPNpsNFbkj3W7XL3SWZVTnni6u3uR9oxx/g16h0zQ9HA7PGaNVhY7hcIjjcrl8jtChVOjr9ZokifF1o9FgjH4UMPJHznq9Zuh4ILDz+XxGo9PpbLdbOvpRdkboaLVaaO92O0QP7zAsle85Jo7j2G63qzloqA7umQ7g4sFgsFgszFONx2PvMCmvptMpxhe/DqbqcrmU+EVUC73f7/Ekp9MJGtXrdbSbzWZcaON90Ov1HNExZxTaD2SCXtI2D7ZarSJCA0wJTu3ZMpME8GZQaBcY2Wy5OFKK9CGhi2rGr311ofHiO4ECp+bZitFWpAzFFgkpGoTWlXXM5/PZbGb3TCYTSUVCVyHx8PYj95D8hHsdX7I6qbyFfr8vcxBJuuP9SBYp9A14Wfxri4Ugiwbql3+9x6RFaJR/kLIotO3xiKlDGyamoW63pPQiJfSpydVwtJdEWQxDybKqxVCF0CarQ/IbGoDisJjG2ekd7uC9J9M7t0iJl3Cimj3MFtq5HMYXO9uFzEsLbUptb+3njQNiXuenZAyA39GJo2Tfekrwmobc2axX8e1QJMvH49GMd1ZFBAdci09Ho5EjvdLt7FKmXdz3TaQOtHfvEN9hXnvrA52lF7q6HB3abv4RKAJh6jQHL4fUhHqoVZ6IJIcFy0tDoSk0hSa/gH9yfKS4/JMjIYQQcuNTgAEADEkRuzoXa8wAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/HydraulicResistor.png")}));
      end HydraulicResistor;
     annotation(classOrder={"*","BaseClasses","Internals","Testers"});
    end HydraulicResistors;

    package JunctionElements "JunctionElements"
      extends TIL.Internals.ClassTypes.ComponentPackage;

      model VolumeJunction "Volume junction"

        /*********************** SIM ***********************************/

        parameter TILMedia.GasTypes.BaseGas gasType=sim.gasType1 "Gas type"
          annotation (Dialog(tab="SIM", group="SIM"), choices(choice=sim.gasType1
              "Gas 1 as defined in SIM", choice=sim.gasType2
              "Gas 2 as defined in SIM",
          choice=sim.gasType3 "Gas 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";
        parameter Boolean generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal
          annotation (Evaluate=true);
        /******************** Connectors *****************************/
      public
        TIL.Connectors.GasPort portA(m_flow(final start=m_flowStart), final gasType=
              gasType) "Fluid port A" annotation (Placement(transformation(extent={{-50,
                  -10},{-30,10}}, rotation=0)));
        TIL.Connectors.GasPort portB(m_flow(final start=-m_flowStart), final gasType=
              gasType) "Fluid port B" annotation (Placement(transformation(extent={{-10,
                  30},{10,50}}, rotation=0)));
        TIL.Connectors.GasPort portC(m_flow(final start=-m_flowStart), final gasType=
              gasType) "Fluid port C" annotation (Placement(transformation(extent={{
                  30,-10},{50,10}}, rotation=0)));

        /****************** Gas *************************/

      protected
        TILMedia.Gas_ph gas(
          final p=p,
          final h=h,
          final xi=xi,
          final gasType=gasType,
          computeTransportProperties=false) "inflowing moist air"
                                                       annotation (Placement(
              transformation(extent={{-10,-10},{10,10}}, rotation=0)));

        /****************** General parameters *******************/
      public
        parameter Modelica.SIunits.Volume volume;

      protected
        parameter TIL.Internals.JunctionFlowType flowType="allow reverse flow";

        /****************** Start values *******************/
      public
        parameter SI.MassFlowRate m_flowStart=0.5 "Start value for mass flow rate"
          annotation (Dialog(tab="Start Values"));

        /****************** Initial values *******************/

        parameter SI.Pressure pInitial=1.013e5 "Initial value for air pressure"
          annotation (Dialog(group="Initial Values"));
        parameter Boolean fixedInitialPressure=true
          "if true, initial pressure is fixed"
          annotation (Dialog(group="Initial Values"));

        parameter SI.Temperature TInitial=298.15 "Initial value for air temperature"
          annotation (Dialog(group="Initial Values"));

        parameter Real[gasType.nc] mixingRatioInitial=gasType.defaultMixingRatio
          "Initial value for mixing ratio" annotation (Dialog(group="Initial Values"));

        final parameter SI.SpecificEnthalpy hInitial=
            TILMedia.GasFunctions.specificEnthalpy_pTxi(
            gasType,
            pInitial,
            TInitial,
            mixingRatioInitial[1:end - 1]/sum(mixingRatioInitial));

        /****************** Additional values *******************/

        SI.MassFraction xi[gasType.nc - 1];
        SI.SpecificEnthalpy h "Specific enthalpy";
        SI.Pressure p(final start=pInitial,fixed=fixedInitialPressure);

        SI.Mass mass "Gas mass in control volume";

        Real drhodt;

      initial equation
        h = hInitial;
        xi = mixingRatioInitial[1:end - 1]/sum(mixingRatioInitial);
      equation
        portA.xi_outflow = xi;
        portB.xi_outflow = xi;
        portC.xi_outflow = xi;

        portA.h_outflow = h;
        portB.h_outflow = h;
        portC.h_outflow = h;

        portA.p = p;

        if (generateEventsAtFlowReversal) then
          der(h) = 1/mass*(portA.m_flow*(actualStream(portA.h_outflow) - h) + portB.m_flow
            *(actualStream(portB.h_outflow) - h) + portC.m_flow*(actualStream(portC.h_outflow)
             - h) + volume*der(p)) "Energy balance";

          der(xi) = 1/mass*(portA.m_flow*(actualStream(portA.xi_outflow) - xi) +
            portB.m_flow*(actualStream(portB.xi_outflow) - xi) + portC.m_flow*(
            actualStream(portC.xi_outflow) - xi)) "Mass balance";
        else
          der(h) = 1/mass*noEvent(portA.m_flow*(actualStream(portA.h_outflow) - h) +
            portB.m_flow*(actualStream(portB.h_outflow) - h) + portC.m_flow*(
            actualStream(portC.h_outflow) - h) + volume*der(p)) "Energy balance";

          der(xi) = 1/mass*noEvent(portA.m_flow*(actualStream(portA.xi_outflow) - xi)
             + portB.m_flow*(actualStream(portB.xi_outflow) - xi) + portC.m_flow*(
            actualStream(portC.xi_outflow) - xi)) "Mass balance";
        end if;

        //______________ Balance equations _______________________

        mass = volume*gas.d "Mass in cv";

        drhodt = gas.drhodh_pxi*der(gas.h) + gas.drhodp_hxi*der(gas.p) + gas.drhodxi_ph
          *der(gas.xi);

        drhodt*volume = portA.m_flow + portB.m_flow + portC.m_flow "Mass balance";

        portA.p - portB.p = 0 "Momentum balance";
        portA.p - portC.p = 0 "Momentum balance";

        annotation (
          defaultComponentName="junction",
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-40,-40},{40,40}},
              initialScale=0.1), graphics={Bitmap(extent={{-40,-20},{40,40}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAFAAAAA8CAYAAADxJz2MAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAJpJREFUeNrs2UsKgDAMQMFUPJg379GqxZ2CUJDgZx5kK3SI3TQip7pNS56acbApBBAgwMELsdZord06/Zs20AYCFECAAAEKIECAAAUQIECAAggQIEABBPjI5thf8b/YknE2GwgQ4Ksrx3uiv/LrAqwUG+gXBghQAAECBCiAAAECBCiAAAECFECAAAFqrNOrnGwgQICSpF+0CjAArOhlS41TRxQAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/JunctionElement.png")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-40,-40},{40,40}},
              initialScale=0.1), graphics),
          Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>p, h, xi</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>isobaric</td>
        </tr>
        </table>
        <p>
        The fixed volume in the junction has three differential states, which are the pressure, the enthalpy and the mass fraction. 
        The volume junction has a dynamic energy balance and a dynamic mass balance.
        </p>
        <hr>
        </html>"));
      end VolumeJunction;
      annotation(classOrder={"*","BaseClasses","Internals","Testers"});
    end JunctionElements;

    package Sensors "Sensors"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model Sensor_T "Sensor reads temperature T [K]"
        extends TIL.GasComponents.Sensors.BaseClasses.PartialSensor;

      /******************** Gas object ********************************/
      protected
        TILMedia.Gas_ph gas(p = port.p,
          h = inStream(port.h_outflow), xi = inStream(port.xi_outflow),
          final gasType = gasType,
          computeTransportProperties=false) "Gas object"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));

      /******************** Parameters ********************************/
      public
        parameter SI.Temperature initialSensorValue = 0
          annotation(Dialog(enable=(useTimeConstant)));

      initial equation

      if useTimeConstant then
        sensorValue = initialSensorValue;
      end if;

      equation
        sensorValue_ = gas.T;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                  {40,40}}), graphics={
              Text(
                extent={{-100,80},{100,40}},
                lineColor={0,0,0},
                textString=DynamicSelect("", String(sensorValue, significantDigits=4) + " K")), Bitmap(
                extent={{-30,-40},{30,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAADwAAAA8CAIAAAC1nk4lAAAABnRSTlMA/wAAAACkwsAdAAAACXBIWXMAAAsTAAALEwEAmpwYAAADFElEQVR42u2a27WiMBSGY9a8kw6gA9IB6UA7ICXQgXQCVgAdnFgBmQ5iBdEKOA/OOHEDMVyEuM7s5ZuYfP7sXPZl16K59ltKIYSUUiklhOh9hjEWRRGllDEWUzp3ynbqp65rzjkhZOyMhBDOeV3Xk6ee8ps8z6Momv2GUBRFeZ6/HTrP8wnSvhR+LPrO0afPQnDOlVJDDyRJcnfc7ktQSt3d/Xw+W1QvyzJhbDGfzrKs97dBEKRpWlVV62xVVaVpGgRB74BZlkkptdbT3UNrrZSifYs9DMOiKNoZVhRFGIbdkSmldV3buW3EUsquBwdBMBMXoHdVJ4TYuccR7/d7rXW7qGmt9/v9KG5X4mUFdpHcwt0DrZTqEjdN077Zmqbp5XaCBisvjuMViB/ccRyDdfkaGuxu62hs1zvLMhs0uO6sTzzELYQYhAaH2VtX3st1Cc7Lfug8z8Hu1m5qYB807yf/oM0dIwiCxffjCfu36SSEEAgNZN7QMSxO8hC7x5vDMHQZcf6lNEmSl7OY95OHZ/+JQSbIvA40EPt+1qAWIc656c2O724daODZnPMWoV/oWenD4eA45fF4HPrKvO+naToUm7nEbISQw+FwOp0eShcIISml+dCoG/2Qmf/n6+tr5mhVVZmEUkoMTkF3pVczgCSEwKbSSZIgL80Ek1JiM1ZljnHl6maCKaWe3GORbMY7zAQTQuCxy3lzaIQQRh9onw/trXvYoC1Zr//u8TOUBmAY7Nv+QzPGMNi3/YQGJyA2UzOW/PG2ZoJRSjG4b4AoxgcDSIwxHFNqxuGeQxNCYkoxuLD6Bn29XruBFYS+3W5lWXol8+12gwHB5BTCOuFWbwoBP6Lcx3eXy8UTscuyvFwuZigOq1sLpsUWUdqSFsNmGtj0bFP7TYxzbnrzU+L8Hane+Uq7pnoXTKoXRZH8tQkjjEuqf2T5wodCESjjOhWKfCvJKaU+rPg5VNz3t8xsaUfwtKBvb6CY1ToxWXWttaV1Qik1sXViqyaVjduBCCHX6xVt1Q7kW+PVD2hx86SZcPeJbZvfgLuCmypm6lkAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Sensor_T.png")}));
      end Sensor_T;

      package BaseClasses
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialSensor

        /*********************** SIM ***********************************/

          parameter TILMedia.GasTypes.BaseGas           gasType = sim.gasType1
            "Gas type" annotation (Dialog(tab="SIM",group="SIM"),choices(
            choice=sim.gasType1 "Gas 1 as defined in SIM",
            choice=sim.gasType2 "Gas 2 as defined in SIM",
            choice=sim.gasType3 "Gas 3 as defined in SIM"));
        protected
          outer SystemInformationManager sim "System information manager";

        /******************** Connector *****************************/
        public
          TIL.Connectors.GasPort port(final gasType = gasType) "Port"
            annotation (Placement(transformation(
                origin={0,-30},
                extent={{-10,-10},{10,10}},
                rotation=180), iconTransformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={0,-40})));

        /******************** Time Constant ************************/

          parameter Boolean useTimeConstant = false
            "= true, if time constant tau is used";

          parameter Modelica.SIunits.Time tau = 1
            "Time constant for delay time of the sensor value"
            annotation(Dialog(enable=(useTimeConstant)));

          Real sensorValue_;

          Modelica.Blocks.Interfaces.RealOutput sensorValue "Sensor value"
            annotation (Placement(transformation(
                origin={0,30},
                extent={{-10,-10},{10,10}},
                rotation=90), iconTransformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,20})));

        equation
        //  assert(tau>0,"Time constant tau must be greater than 0");

          port.m_flow = 0.0;
          port.h_outflow = 0.0;
          port.xi_outflow = zeros(gasType.nc-1);

           if useTimeConstant then
             sensorValue = sensorValue_ - der(sensorValue)*tau;
           else
            sensorValue = sensorValue_;
           end if;

          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                    {40,40}}),graphics),
                               Icon(coordinateSystem(preserveAspectRatio=false,extent={{-40,-40},
                    {40,40}}),      graphics={
                Text(visible=useTimeConstant,
                  extent={{10,50},{50,10}},
                  lineColor={0,0,0},
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid,
                  fontName="Symbol",
                  textString="t")}));
        end PartialSensor;
      end BaseClasses;
    annotation(classOrder={"*","Internals","BaseClasses","Testers"});
    end Sensors;

    package Volumes "Volumes"
      extends TIL.Internals.ClassTypes.ComponentPackage;

      model Volume "Finite volume"

        /*********************** SIM ***********************************/

        parameter TILMedia.GasTypes.BaseGas gasType=sim.gasType1 "Gas type"
          annotation (Dialog(tab="SIM", group="SIM"), choices(choice=sim.gasType1
              "Gas 1 as defined in SIM", choice=sim.gasType2
              "Gas 2 as defined in SIM",
          choice=sim.gasType3 "Gas 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";

        /******************** Connectors *****************************/
      public
        TIL.Connectors.GasPort portA(m_flow(final start=m_flowStart), final gasType=
              gasType) "Gas port A" annotation (Placement(transformation(extent={{-50,
                  -30},{-30,-10}}, rotation=0), iconTransformation(extent={{-50,-50},
                  {-30,-30}})));
        TIL.Connectors.GasPort portB(m_flow(final start=-m_flowStart), final gasType=
              gasType) "Gas port B" annotation (Placement(transformation(extent={{30,
                  -30},{50,-10}}, rotation=0), iconTransformation(extent={{30,-50},{
                  50,-30}})));
        TIL.Connectors.HeatPort heatPort "Heat port" annotation (Placement(
              transformation(extent={{-10,70},{10,90}}, rotation=0),
              iconTransformation(extent={{-10,50},{10,70}})));

        /****************** Gas *************************/

      protected
        TILMedia.Gas_ph gas(
          final p=p,
          final h=h,
          final xi=xi,
          final gasType=gasType,
          computeTransportProperties=false) "Gas object"
                                              annotation (Placement(transformation(
                extent={{-10,-10},{10,10}}, rotation=0)));

        /****************** General parameters *******************/
      public
        parameter Modelica.SIunits.Volume volume;

      protected
        parameter TIL.Internals.CellFlowType cellFlowType="allow reverse flow";

        /****************** Start and initial values *******************/
      public
        parameter SI.MassFlowRate m_flowStart=0.5 "Start value for mass flow rate"
          annotation (Dialog(tab="Start Values"));

        parameter SI.Pressure pInitial=1.013e5 "Initial value for air pressure"
          annotation (Dialog(group="Initial Values"));
        parameter Boolean fixedInitialPressure=true
          "if true, initial pressure is fixed"
          annotation (Dialog(group="Initial Values"));

        parameter SI.Temperature TInitial=298.15 "Initial value air temperature"
          annotation (Dialog(group="Initial Values"));

        parameter Real[gasType.nc] mixingRatioInitial=gasType.defaultMixingRatio
          "Initial value for mixing ratio" annotation (Dialog(group="Initial Values"));

        final parameter SI.SpecificEnthalpy hInitial=
            TILMedia.GasFunctions.specificEnthalpy_pTxi(
            gasType,
            pInitial,
            TInitial,
            mixingRatioInitial[1:end - 1]/sum(mixingRatioInitial));

        /****************** Additional values *******************/

        SI.Mass mass "Gas mass in control volume";

        Real drhodt;

        SI.Pressure p(start=pInitial,fixed=fixedInitialPressure);
        SI.MassFraction xi[gasType.nc - 1];
        SI.SpecificEnthalpy h "Specific enthalpy";

      initial equation
        h = hInitial;
        xi = mixingRatioInitial[1:end - 1]/sum(mixingRatioInitial);
      equation

        p = portA.p;

        portA.h_outflow = h;
        portB.h_outflow = h;

        portA.xi_outflow = xi;
        portB.xi_outflow = xi;

        heatPort.T = gas.T;

        //______________ Balance equations _______________________

        der(xi) = 1/mass*(noEvent(actualStream(portA.xi_outflow))*portA.m_flow -
          portA.m_flow*xi + noEvent(actualStream(portB.xi_outflow))*portB.m_flow -
          portB.m_flow*xi) "Gas mass balance";

        der(h) = 1/mass*(noEvent(actualStream(portA.h_outflow))*portA.m_flow - portA.m_flow
          *h + noEvent(actualStream(portB.h_outflow))*portB.m_flow - portB.m_flow*h
           + heatPort.Q_flow + volume*der(p)) "Energy balance";

        mass = volume*gas.d "Mass in cv";

        drhodt = gas.drhodh_pxi*der(gas.h) + gas.drhodp_hxi*der(gas.p) + gas.drhodxi_ph
          *der(gas.xi);

        drhodt*volume = portA.m_flow + portB.m_flow "Mass balance";

        portA.p - portB.p = 0 "Momentum balance";

        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-40,-80},{40,80}},
              initialScale=0.1), graphics={
              Bitmap(extent={{-40,-80},{40,60}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/ContainerVolume.png"),
              Text(
                extent={{-70,-70},{-30,-110}},
                lineColor={0,0,0},
                textString="A"),
              Text(
                extent={{30,-70},{70,-110}},
                lineColor={0,0,0},
                textString="B")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-40,-80},{40,80}},
              initialScale=0.1), graphics),
          Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>p, h, xi</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>isobaric</td>
        </tr>
        </table>
        <p>
        The fixed volume has three differential states, which are the pressure, the enthalpy and the mass fraction. 
        The volume has a dynamic energy balance and a dynamic mass balance.
        A heat flow rate can be considered, if the heat port is connected.
        Otherwise the heat flow rate is zero.
        </p>
        <p>
        <a href=\"../Documentation/Antoine.pdf\">Equation of Antoine: Vapor Pressure of water</a><br>
        <a href=\"../Documentation/effectiveheattransfer.pdf\">Effective heat transfer coefficient relating to the inlet temperature</a>
        </p>
        </html>"));
      end Volume;
       annotation(classOrder={"*","BaseClasses","Internals","Testers"});
    end Volumes;
    annotation(classOrder={"*","Internals","Testers"});
  end GasComponents;

  package VLEFluidComponents "Component models using VLE fluid mixtures"
  extends TIL.Internals.ClassTypes.ComponentPackage;

    package Compressors "Compressors"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      package BaseClasses
      import TIL;
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialEffCompressor "Efficiency Based Compressor Model"
          extends PartialCompressor;

        equation
          portB.m_flow + portA.m_flow = 0 "mass balance";

          portA.h_limit = -1e6; //no cell volume, ignore singularity approach
          portB.h_limit = -1e6;

          portA.xi_outflow = inStream(portB.xi_outflow);
          portB.xi_outflow = inStream(portA.xi_outflow);

          simPort.vleFluidMass   = 0.0;
          simPort.vleFluidVolume = 0.0;

          portA.h_outflow = inStream(portB.h_outflow);//no change of enthalpy at flow reversal

          assert(portB.p > portA.p, "The compressor cannot handle flow reversal but is experiencing a negative pressure ratio!");

          //____________________ Efficiencies ___________________________

          dischargeVLEFluid_h =  (isentropicDischargeVLEFluid.h - suctionVLEFluid.h)/isEff + suctionVLEFluid.h;

          portA.m_flow = volEff*suctionVLEFluid.d*n*displacement*relDisplacement;

          shaftPower = portA.m_flow*(isentropicDischargeVLEFluid.h - suctionVLEFluid.h)/effIsEff;

        end PartialEffCompressor;

        partial model PartialCompressor
          "Base class for positive displacement compressors"

        /*********************** SIM ***********************************/

          parameter TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType = sim.vleFluidType1
            "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
            choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
            choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
            choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));
        protected
          outer SystemInformationManager sim "System information manager";

          TIL.Internals.SimPort simPort;

         /******************** Ports *****************************/
        public
          TIL.Connectors.RotatoryFlange rotatoryFlange if use_mechanicalPort annotation (Placement(
                transformation(extent={{70,-10},{90,10}}, rotation=0)));
          TIL.Connectors.VLEFluidPort portA(final vleFluidType=vleFluidType)
            "Suction port"
            annotation (Placement(transformation(extent={{-10,-90},{10,-70}}, rotation=
                    0)));
          TIL.Connectors.VLEFluidPort portB(final vleFluidType=vleFluidType)
            "Discharge port"
            annotation (Placement(transformation(extent={{-10,70},{10,90}}, rotation=0)));

           Modelica.Blocks.Interfaces.RealInput relDisplacement_in if use_relDisplacementInput
            "relative displacement input"
            annotation (Placement(transformation(
                origin={110,-60},
                extent={{-10,-10},{10,10}},
                rotation=180)));

         /******************** VLEFluids *****************************/
        protected
          TILMedia.VLEFluid_ph suctionVLEFluid(p=portA.p, h=inStream(portA.h_outflow),
            xi=inStream(portA.xi_outflow), final vleFluidType=vleFluidType)
            annotation (Placement(transformation(extent={{-40,-100},{-20,-80}},rotation=0)));
          TILMedia.VLEFluid_ps isentropicDischargeVLEFluid(p=portB.p, s=suctionVLEFluid.s,
            xi=suctionVLEFluid.xi, final vleFluidType=vleFluidType)
            annotation (Placement(transformation(extent={{-40,80},{-20,100}}, rotation=0)));
          TILMedia.VLEFluid_ph dischargeVLEFluid(p=portB.p, h=dischargeVLEFluid_h,
            xi=suctionVLEFluid.xi, final vleFluidType=vleFluidType)
            annotation (Placement(transformation(extent={{20,80},{40,100}}, rotation=0)));

         /****************** General parameters *******************/

        public
           parameter Boolean use_mechanicalPort=false
            "= true, if mechanical port is used" annotation(Evaluate = true, Dialog(group="Mechanics"));

           parameter SI.Frequency nFixed(min=0)=20 "Speed"
            annotation(Dialog(enable= not use_mechanicalPort, group="Mechanics"));

           parameter SI.Volume displacement=30e-6 "Displacement" annotation(Dialog(group="Mechanics"));

           parameter Boolean use_relDisplacementInput=false
            "= true, if relative displacement input connector is used" annotation(Evaluate = true, Dialog(group="Mechanics"));

        /************************* Additional variables ***************/

          SI.Frequency n "Speed";

          SI.Efficiency volEff "Volumetric efficiency";
          SI.Efficiency isEff "Isentropic efficiency";
          SI.Efficiency effIsEff "Effective isentropic efficiency";

          TIL.Utilities.Units.RelativeVolume relDisplacement
            "relative Displacement [0,1] (=1 if input not used)";

          SI.Power shaftPower "Shaft power";
          Modelica.SIunits.SpecificEnthalpy dischargeVLEFluid_h;

        protected
          TIL.Internals.GetInputsRotary getInputsRotary
                                          annotation (Placement(transformation(extent={{60,-10},
                    {40,10}},            rotation=0)));

          Modelica.Blocks.Interfaces.RealOutput
            getInputsRelDisplacement
            annotation (Placement(transformation(extent={{-34,-52},{-14,-32}}, rotation=
                   0)));

          Modelica.Blocks.Sources.Constant constRelDisplacement(k=1) if not use_relDisplacementInput
            annotation (Placement(transformation(extent={{-82,-52},{-62,-32}})));

        protected
          SI.AngularVelocity w "Angular velocity";
          SI.Torque tau "fluidTorque";

          /*******************Summary**********************/
        public
          parameter Boolean includeDefaultSummary = true
            "Include summary record in model results"
         annotation(Dialog(tab="Advanced", group="Summary"));

        protected
          record Summary
            extends TIL.Internals.ClassTypes.Record;

            SI.Pressure p_A "Pressure at port A";
            SI.Pressure p_B "Pressure at port B";
            SI.Temperature T_A "Temperature at port A";
            SI.Temperature T_B "Temperature at port B";
            SI.Temp_C T_degC_A "Temperature at port A";
            SI.Temp_C T_degC_B "Temperature at port B";
            SI.SpecificEnthalpy h_A "Specific enthalpy at port A";
            SI.SpecificEnthalpy h_B "Specific enthalpy at port B";
            SI.Density d_A "Density at port A";
            SI.Density d_B "Density at port B";
            SI.MassFlowRate m_flow_A "Mass flow rate at port A";
            SI.MassFlowRate m_flow_B "Mass flow rate at port B";
            SI.TemperatureDifference superheating "Superheating";
            SI.MassFraction q_A "Steam mass fraction (quality) at port A";
            SI.MassFraction q_B "Steam mass fraction (quality) at port B";
            Real pressureRatio "Pressure ratio";
            SI.Power P_vle "VLE fluid power";
            SI.Power P_shaft "Shaft power";
            SI.HeatFlowRate Q_flow_housing "Housing heat flow rate";
            Real volEff "Volumetric efficiency";
            Real effIsEff "Effective isentropic efficiency";
            Real isEff "Isentropic efficiency";
            SI.Frequency speed "Speed of compressor";
            Modelica.SIunits.Conversions.NonSIunits.AngularVelocity_rpm speed_rpm
              "Speed of compressor";
            SI.Volume displacement "Displacement";
            TIL.Utilities.Units.RelativeVolume relDisplacement "Relative displacement";
            SI.Mass mass_vle "Total fluid mass";
            SI.Volume volume_vle "Total fluid volume";

          end Summary;

          replaceable record SummaryClass = Summary;

        public
          SummaryClass summary(
            p_A = portA.p,
            p_B = portB.p,
            T_A = suctionVLEFluid.T,
            T_B = dischargeVLEFluid.T,
            T_degC_A = suctionVLEFluid.T - 273.15,
            T_degC_B = dischargeVLEFluid.T - 273.15,
            h_A = suctionVLEFluid.h,
            h_B = dischargeVLEFluid.h,
            d_A = suctionVLEFluid.d,
            d_B = dischargeVLEFluid.d,
            m_flow_A = portA.m_flow,
            m_flow_B = portB.m_flow,
            superheating = suctionVLEFluid.T -suctionVLEFluid.VLE.T_v,
            q_A = suctionVLEFluid.q,
            q_B = dischargeVLEFluid.q,
            pressureRatio = portB.p/portA.p,
            P_vle = -(portA.m_flow*suctionVLEFluid.h + portB.m_flow*dischargeVLEFluid.h),
            P_shaft = shaftPower,
            Q_flow_housing = -(shaftPower + portA.m_flow*suctionVLEFluid.h + portB.m_flow*dischargeVLEFluid.h),
            volEff = volEff,
            effIsEff = effIsEff,
            isEff = isEff,
            speed = n,
            speed_rpm = n*60,
            displacement = displacement,
            relDisplacement = relDisplacement,
            mass_vle = simPort.vleFluidMass,
            volume_vle = simPort.vleFluidVolume) if includeDefaultSummary
            annotation (Placement(transformation(extent={{40,38},{60,58}},  rotation=0)));

        equation
          portB.h_outflow = dischargeVLEFluid_h;

          w = 2*PI*n;

          connect(sim.fluidPort[vleFluidType.ID],simPort.vleFluidPort);

        //____________________ Port Handling ___________________________

          if use_mechanicalPort then
            der(getInputsRotary.rotatoryFlange.phi) = w;
            tau = getInputsRotary.rotatoryFlange.tau;
            shaftPower = tau*w;
          else
            n = nFixed;
            getInputsRotary.rotatoryFlange.phi = 0.0;
            tau = shaftPower/max(w,1e-6);
          end if;

          relDisplacement = getInputsRelDisplacement;

          connect(getInputsRotary.rotatoryFlange, rotatoryFlange) annotation (Line(
              points={{60,0},{80,0}},
              color={135,135,135},
              thickness=0.5));
          connect(relDisplacement_in, getInputsRelDisplacement) annotation (Line(
              points={{110,-60},{-44,-60},{-44,-42},{-24,-42}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(constRelDisplacement.y, getInputsRelDisplacement) annotation (Line(
              points={{-61,-42},{-24,-42}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (defaultComponentName="compressor",Icon(coordinateSystem(
                  preserveAspectRatio=true, extent={{-80,-80},{80,80}}),
                                                             graphics={Bitmap(extent={{
                      -80,-80},{80,80}},
                  imageSource=
                      "iVBORw0KGgoAAAANSUhEUgAAAKAAAACgCAYAAACLz2ctAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAADl5JREFUeNrsXe1V47oWNaz3H6YC0gFMBaGDcCsIHZgOQgfJVBBuBYEKHCpIqCChgkAFPG9GzjUnki3Z+rJ99lpaWQNDLEvb52tL8lnCKOM6b6O83Yh/34rPkWgm2IsGbPP2IT7xszce6r84G/C9jwXRbgnpfGFL2isTsP+Euy21GLEutdeE0Wlc5G2at2XeDnn76lg7iL5Pxb2wBewIJnm7z9td0y+4urpKRqNRcnNzk1xeXv4NBm/NjOZ6vf4bCO73x/b+/t7mvp7z9pS3FyZgfLjK24Mg3qW2iby4+CYZyIXPgnROA7/t9puM+ARJ8fn5+WnyFR+CiIu8vSeM4NYu03VrOeG+JpPJ13w+/9psNl+xAH1Bn9A39NHATWdiDBiegbhopzNJuTv9StP0K8uyr64AfUWf0XdNIu7EmDBiIR4aLErXgXswsIidI+J5x4j3lBgUhO/u7jr/xBnew0iMEVtEi7W7rC6ug7tarVYnvzscDp23gLgHel+41+l0qhMvZmIMGQ1qeMs64s1msyPJEMSXf399ff3VF+BeyveGey3IiTHQIOKyz7VEF5ntQZd4BcbjsXSS+oC6h0uTiAfOmOut3qrqSZYRT+WmYiqz2CjT6IQXBRFrrOGKraGh1UMmuNvtlBO0XC5PSi99Ay3J4J5VwFjVZM9sDUtWb15Vw9Op3yEgL/8dkpK+AfdUvkfcs049saaWOB+yNYR8tlENDgZcN4ulsQ+yxL6BZvm4Z90smpKXtI2YC3a5Jlav/JQ3mZgugj5opuNUYQ0H5ZJT1dMIt2Jau6NPdx/UD11VxDTUwNjScIW0tO/kW6pKK1VBta3gvOuwlWzheypKNstBlVgwiE1LJsj2+qh+mKgiVdWButJOhUvuVanmQpVsoHjchjB9Vj9MVZGmhKYFfJKcXPSWfDplhDr0Wf3QfegwBm1RERd2moRK8tkgSt/Vj7aqSFtid52ESvLZShKGoH74TrzomPog4bkj8q0TyT7b/AaT+/t7KxcpNv0UMN001GXQNYJ0LJoCc4M5kuBGzGknLOHKpeUbkvphWxWxYAlXnazz2SYfVT/QhgZ6/7b3vFSQMNo6YeqDfENTP1ypIi1JGJ1iMnGV7Q5d/QidhFWsL4xGO76SLSywUefTVT+aqgFdhs9xUNQJDzGsopGWW2wUR1n98KuKmBb9bZVn2pZhHmm5BeeqPD8/O2M8/e4+bL20VY5xPe6YW0l55jGauA/lAJdqxFDVD9+qSNX1FKtoJiFc78FHxsvqR1wJmSIzPjR1xU1d8FNCTqHKA1VrKgerH/qgY2BLFalSSzDXBJeCE2FcL55CH+vwhqx+hFJFVKGQYi3hJIjr9XHyFKsf4VQR3flo4opNXfCCut40Tb24QprhTSa8rVU1Fi6z4bLrx9xLXPHC1TXHoVyvrOY1RPUjtuSswhU7ORApC2HqWf2Ie3wUrjizTb5pEvDwR1Y/4lJFKBTHgExtxoAn1e7FYuEtxqGlhSGrH7qqiOtyjAYXHp1ZP6yQ8AVWP+JURSgUq2asnNK6o3UmnzdG61ysfuirIj7rpOCERKbbtXXBYPCo/IOHh4fjy1t8gJYUWP3QV0V8lGOO9ZecE+AGwaitFQxq/Vj9iF8VsWEFtSU3n7GfKq5hmKkivuNlRSzYSDXIQls/3vvRviTi+7BOhRXMTGNArDy8pWm+z9iPyy/dK8cUsaBknm4Tw+X789DKA6sf3R03WR8Ep7Qt4D0Vu/EmSZ+gGdz19bX3PnQRGCOMVahsuOiDZLHIvS4BJ3TFi+uFpjrul8sv3XHDCs5c6iYjqyTwmcusfnRbFVGV0BLJ0R7UAmIx4V1s1q94sTRDDxgrjFmEVvAuIQtWzyX/IQlNQN56ad8N+44DK7hTOZnLGHRXVj+6r4qo9OmEHG5UaQFDWJ7tdpt8fn5yAmJZF8aYYmxDW2LKsTIBxzT7DUHAp6enhJaAfBfA+wCMGS2F0LENRMDLpLRk/5xUq38E/iEsD5df3FnBEIkI+kATojLXlAQMMfH7/T55e3vjBMSR9cHYYoxDPwjREpDVD7uIQRXRJeBY44/Y/bIbtkXAE86lCasfrIr4VUXSsgX8ITOEUB1Y/XCDWFQRyVzeKAkYQ/zHyYe7ZCSSOPAmagvIBOwXAass4LUse/IJVOjf3985AfFkfUKoIgpOXYOAIw22enW/rH7YhUwV8W0FFZy6PE8kh4yHjv/Y+rm3giHcsIRbtyDgZUj3y+pHmDgwhCoi49Z56ASEJh+sfribfKqK+C7HSOb19lwWL7D7ZTfs0wIGNTcvLy8//h1iBfZQQMeWjn0Iw3xCQJ8WiD6BrH64z0SpKuLTCkq4NToPOSCsfoRPRkJkw9QFBwOrH+EJGEIXjoKArH7EkYhgDkLsFQlOQGr6x+Mxqx8eEIMqEiUB2f2Gs4JREdCHFWL1I644MNReESkBPz4+vCcf0AhZ/fAHjDXVZUMlI0FcMLvf+KxgKDcchIC0As8EDE/AUKqIdwLK1A8uv4RJREKqItEQkK3fsN0wCLj3GYzy3t+4rGDIuQf3TgjoEjL1gy1gPBYwgCqy91qGYfUjLmDsMQe+3LCs1ggCrqmV4viP40BPBFyf67DU1sVZ/YifgC5VEZUF3NI4wEcAyupHHPCpiki4tZYmIS7cMLvfYbthBac+QMA3H26YF592h4AuVBEFp96KGHDr0gLiiSofPM7qR1zwoYpIOLUtYsAkcVyMZuvXPSvomgOUgGvXFpA+cYz4rGAIC1gA1Ugnp5PGckInI9wJtTIOCM4dLeCrhsm0YnpZ/YgTMlXEFQfKnCsXotcuLk5fjsLxX3fiQFsvtpFwSUquWWL5oHKZWee3nnfrbes2wiXJAeWzgnRKC4iySVsrSANZVj/ihkwVaZuMgEP03X9lrp0Tn/xh8+KsfnTfDdvmgODYq+r/W31dKzW9WZaxn4scmCOboVjd61opprZS8VjeV8sI975mRfllWiYcXY71XJfFNs18uPg8PFVEwZ1an76yYbmo6V0ul2xaOgLMlY1QTJL9rnSIO6Fm09QEs/rBqggNwUSbULLJtmW+0GzY1A3LDh5n9aNbqkjbA80Xi4Us+9Ve5zVvU0DOO//jb+fzOZuVjgFzVp5DzGmbgrbglDau6BdMp9PgojbDH9qEUeCKhIDGb0DKaDKi0wFbASwjPJokkuCIJPnIVCSrOppjQaU5iV9n9YNVkZPYTyK9LZr2YWdqBW0VMRnhYSomKKzfrs1DcKKMzGYzVj9YFZEC3KhTPkxcMPBvQvaLwMSqju9g9aN/0D3ACJyQhGh7waHGBAQedWNBjv+GGwcqYr9HW/3Y1dUFWf0YriqiqPtpxX66B1SeMPnh4YHVD1ZFpFywbf2kdcGErO8bj8esfgxQFaHrB+vqfm1wsnUThUqYaFY/hqmKoEkWnB63XOrgfwYExDJqrEq4L36A044eHx9PXrGKfQX82tX+AHOJOS2fboVkRHbireDIq6u+4ACRA2U8XXyQpimbjZ4Bc1o156IdBEe0YXpK/mfZAhagB09y/a//9UA65wL3giPaOGvYH6xslRb5cMqSj9d9McJkxJJa39Er5+0f0+9s+p4QMP1D50lh9NcKloUQmWd0ScBP1QWZgIMkoLHrtYW5qjTD6J8ioii5zEM+FMh4NrRTKEoz+gUqNIi2Mc16XeBKVprRXcLPiB+KJfaHpMEye1eYSDrIclwPoFjjJ91iGRqprKO8Ib27oPt7Si2NNVFaMgl7T75l7Nn6iknYW/Ktkg5AmhkzCTtPvigy3mrmXVwkm82GSdhT8mFu6Qtt4goAl8skv4/vVkXCqp11jOiy3W/yFfOKOY4S0+n0SD4dEnKdMPo63wn5ioa5jgpYrHg4HE4IWEdCVNdZtgsHjL1C4VCSDw1zTQ8zD4osy6TkIySUZsfQF3n5vn9gzBXabpHtXlTNKeY8jupzmlaSjxBRWifE7ntOTvwmG5IjNI51Pt35xNxH63orSJgqbvw7FmGX7NblVsR73wqHyVwGd8V1rreChBPZAobCJfPrHOwDY1rhcjEXkyZzGcwVTyaTRuQrkfBKlZwkYmMTW0M7Vo9uKJIkG1dt5hJc8I7dbteKgKXkZK4aHLaGTq1esZj0ou08ggteMZvNWpNP1yWj5U8Yv+TQABgrjFkF8Rq7XFUDJ7zJbaaJh4E1XFUM2ne1nt1ytbutUDS0SyxNGjjhRaazbf1MrSHKB0xEOfEqSitOrJ53K+jK+ims4bLqSWYiahOvWMN34XrenFtB19ZPQkQccpPVERG1rSHFiLhX3LMG8TB2Y59z5swK+rJ+iko71O9dzWB/65p9PhQd91ah3dLDIaemSlXUVlC22sV1owcj6hIRlgG1rz5ozLgH3IuGtftBvAIYQ9/z5mS1jI26X9vaUtEHXSIWtURMYJfqiegr+lxTwzshXt24hZy7oKpHkzaf/9xwL3uSRcacaU7StxVBfQzbRWOyjugL+oS+aVq6cow3qfMcGEvf82dVHcljj+A3UBXLCFlvXlW+URESMRUyScRXPkiJa+BauCaubUi4opwyr5LPaCIQwoCAMzo400k+fB+3huv9+vXrx8+wH0Hn1NWzs7OJOCyn8TsisMJjNBodG2B66FJxkPd+vz82yWmiJsDxZ0/55Na+8hQnl/7+/fvHz5Ac+D40vuY4N/vr/Vw9PXgIGtYSp6IGdjC0MjG0g+j7tEkNj2aiIbyYlfWCsDyhsygbGbioKc5MYsYALRN9HMc4hqZN7JBst+DUd6d9Pb2CkKmwMpsAZNuIa6cuisU2vIiNVrdgtfKU/BCv2kL8QuMGF4de5oODk9xfSfyI9BGBUnHB4nMkmgn2yX/v2VuXPj/ya7+5Hkc6ZhhTjK3vtxeAQ3/+/FH+/v8CDAC/sT+6NRE18gAAAABJRU5ErkJggg==",
                  fileName="modelica://TIL/Images/CompressorUni.png")}),                              Diagram(
                coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),                   graphics));
        end PartialCompressor;
      end BaseClasses;
      annotation(classOrder={"SimpleCompressor","EffCompressor","*","BaseClasses","Internals","Testers"});
    end Compressors;

    package JunctionElements "JunctionElements"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model VolumeJunction "VLEFluid junction with 3 fluid ports"

      /*********************** SIM ***********************************/

        parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
          "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
          choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
          choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
          choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));

      protected
        outer SystemInformationManager sim "System information manager";
        TIL.Internals.SimPort simPort;
      public
        parameter TIL.Internals.PressureStateID pressureStateID=1 "Pressure state ID"
          annotation(Dialog(tab="SIM",group="SIM"));

      /********************** Connectors **************************/

      public
        TIL.Connectors.VLEFluidPort portA(final vleFluidType = vleFluidType)
          "Fluid port 1"
          annotation (Placement(transformation(extent={{-50,-10},{-30,10}}, rotation=
                  0)));
        TIL.Connectors.VLEFluidPort portB(final vleFluidType = vleFluidType)
          "Fluid port 2"
          annotation (Placement(transformation(extent={{-10,30},{10,50}}, rotation=0)));
        TIL.Connectors.VLEFluidPort portC(final vleFluidType = vleFluidType)
          "Fluid port 3"
          annotation (Placement(transformation(extent={{30,-10},{50,10}}, rotation=0)));

      /*********************** VLEFluid ***************************/
      protected
        TILMedia.VLEFluid_ph vleFluid(p=portA.p,h=h,xi=xi,final vleFluidType = vleFluidType,
          computeTransportProperties=false) "VLEFluid in mixing volume"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));

      /******************** General Parameters ********************/
      protected
        parameter Boolean generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal  annotation(Evaluate=true);
        parameter Boolean removeNLSystems=sim.removeNLSystems annotation(Evaluate=true);
        parameter Boolean removeSingularity=sim.removeSingularity annotation(Evaluate=true);

      public
        parameter SI.Volume volume=1e-7 "|Geometry|Mixing volume";
        parameter SI.SpecificEnthalpy hInitial=300e3
          "|Initial values|Initial value for enthalpy";

        parameter Real[vleFluidType.nc] mixingRatioInitial=vleFluidType.defaultMixingRatio
          "|Initial values|Initial value for mass fraction";

      /************************* Additional variables ***************/

        SI.Mass mass "VLEFluid mass";
        Real drhodt "Derivative of density wrt time";

        SI.SpecificEnthalpy h "Specific enthalpy";
        SI.MassFraction[vleFluidType.nc-1] xi "Mass fraction";

      protected
        parameter Real yLimit=0.95;

      initial equation
        xi = mixingRatioInitial[1:end-1]/sum(mixingRatioInitial);
        h = hInitial;

      equation
        assert(sim.dpdtPort[pressureStateID].counter>0.5,"Pressure state ID is not valid");
        connect(sim.fluidPort[vleFluidType.ID],simPort.vleFluidPort);
        simPort.vleFluidMass
                          = mass;
        simPort.vleFluidVolume
                            = volume;

        if (removeSingularity) then
          portA.h_outflow -vleFluid.h = noEvent(max(0,inStream(portA.h_limit)-vleFluid.h));
          portB.h_outflow -vleFluid.h = noEvent(max(0,inStream(portB.h_limit)-vleFluid.h));
          portC.h_outflow -vleFluid.h = noEvent(max(0,inStream(portC.h_limit)-vleFluid.h));
        else
          portA.h_outflow -vleFluid.h = 0;
          portB.h_outflow -vleFluid.h = 0;
          portC.h_outflow -vleFluid.h = 0;
        end if;

        portA.h_limit = vleFluid.h+yLimit*vleFluid.d/vleFluid.drhodh_pxi;
        portB.h_limit = vleFluid.h+yLimit*vleFluid.d/vleFluid.drhodh_pxi;
        portC.h_limit = vleFluid.h+yLimit*vleFluid.d/vleFluid.drhodh_pxi;

        portA.xi_outflow = xi;
        portB.xi_outflow = xi;
        portC.xi_outflow = xi;

      // Momentum balance
        portB.p = portA.p;
        portC.p = portA.p;

        mass = volume*vleFluid.d "Mass in cv";

        if (generateEventsAtFlowReversal) then
          der(h) = 1/mass*(portA.m_flow*(actualStream(portA.h_outflow) - h) +
            portB.m_flow*(actualStream(portB.h_outflow) - h) +  portC.m_flow*(actualStream(portC.h_outflow) - h) + volume*sim.dpdtPort[pressureStateID].dpdt)
            "Energy balance";

          der(xi) = 1/mass*(portA.m_flow*(actualStream(portA.xi_outflow)-xi) +
            portB.m_flow*(actualStream(portB.xi_outflow)-xi) + portC.m_flow*(actualStream(portC.xi_outflow)-xi))
            "Mass balance";
        else
          der(h) = 1/mass*noEvent(portA.m_flow*(actualStream(portA.h_outflow) - h) +
            portB.m_flow*(actualStream(portB.h_outflow) - h) +  portC.m_flow*(actualStream(portC.h_outflow) - h) + volume*sim.dpdtPort[pressureStateID].dpdt)
            "Energy balance";

          der(xi) = 1/mass*noEvent(portA.m_flow*(actualStream(portA.xi_outflow)-xi) +
            portB.m_flow*(actualStream(portB.xi_outflow)-xi) + portC.m_flow*(actualStream(portC.xi_outflow)-xi))
            "Mass balance";
        end if;

        drhodt*volume = portA.m_flow + portB.m_flow + portC.m_flow "Mass balance";

        drhodt = vleFluid.drhodh_pxi*der(vleFluid.h)
               + vleFluid.drhodp_hxi*sim.dpdtPort[pressureStateID].dpdt
               + vleFluid.drhodxi_ph*der(vleFluid.xi);

        annotation(defaultComponentName="junction",
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},{40,40}}),
               graphics={Text(
                extent={{-100,-20},{100,-60}},
                lineColor={153,204,0},
                textString=
                     "(%pressureStateID)"), Bitmap(extent={{-40,-20},{40,40}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAFAAAAA8CAYAAADxJz2MAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAJpJREFUeNrs2UsKgDAMQMFUPJg379GqxZ2CUJDgZx5kK3SI3TQip7pNS56acbApBBAgwMELsdZord06/Zs20AYCFECAAAEKIECAAAUQIECAAggQIEABBPjI5thf8b/YknE2GwgQ4Ksrx3uiv/LrAqwUG+gXBghQAAECBCiAAAECBCiAAAECFECAAAFqrNOrnGwgQICSpF+0CjAArOhlS41TRxQAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/JunctionElement.png")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},{40,40}}),
                  graphics),
              Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>h, xi, derivative of pressure in the PressureStateElement</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>isobaric</td>
        </tr>
        </table>
        <p>
        The fixed volume in the junction has two differential states, which are the enthalpy and the mass fraction. 
        The volume junction has a dynamic energy balance and a dynamic mass balance.
        </p>
        <hr>
          </html>"));
      end VolumeJunction;
    annotation(classOrder={"*","Testers","Internals"});
    end JunctionElements;

    package PressureStateElements "PressureStateElements"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model PressureState "Pressure state model"

       /*********************** SIM ***********************************/

        parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
          "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
          choice=sim.vleFluidType1 "VLE Fluid 1 as defined in SIM",
          choice=sim.vleFluidType2 "VLE Fluid 2 as defined in SIM",
          choice=sim.vleFluidType3 "VLE Fluid 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";
        TIL.Internals.SimPort simPort;
      public
        parameter TIL.Internals.PressureStateID pressureStateID=1 "Pressure state ID"
          annotation(Dialog(tab="SIM",group="SIM"));

       /******************** Connectors *****************************/

        TIL.Connectors.VLEFluidPort portA(final vleFluidType=vleFluidType,
          p(start=pInitial, final fixed=fixedInitialPressure)) "Port A"
          annotation (Placement(transformation(extent={{50,-10},{70,10}}, rotation=0)));
        TIL.Connectors.VLEFluidPort portB(final vleFluidType=vleFluidType,
          p(start=pInitial)) "Port B"
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}}, rotation=
                  0)));

       /******************** General Parameters ********************/

        parameter SI.AbsolutePressure pInitial
          "|Initialization|Initial value for pressure";

        parameter Boolean fixedInitialPressure=true
          "if true, initial pressure is fixed"
          annotation(Evaluate=true,Dialog(tab="Advanced",group="Initialization"));

      protected
       parameter Internals.PressureStateType pressureStateType = "der(p)" annotation(Dialog(tab="Advanced", group="Pressure State Type"),HideResult=True);

       parameter SI.Time timeConstant=0.001
          "filter time constant for calculating der(p) = (pIn-p)/timeConstant activated when p=f(t)"
          annotation(Dialog(tab="Advanced",group="Prescribed Pressure",enable= (pressureStateType == "p=f(t)")),HideResult=True);

       parameter Boolean usePressureInput = false annotation(Dialog(tab="Advanced", group="Pressure State Type",enable= (pressureStateType == "p=f(t)")),HideResult=True);
       parameter Boolean useDpdtInput = false annotation(Dialog(tab="Advanced", group="Pressure State Type", enable= (pressureStateType == "der(p)=f(t)")),HideResult=True);

        model GetInputs "Get enabled inputs and parameters of disabled inputs"
          extends Modelica.Blocks.Interfaces.BlockIcon;

          Modelica.Blocks.Interfaces.RealInput p_in "Prescribed pressure"
          annotation (Placement(transformation(extent={{-140,60},{-100,100}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput dpdt_in
            "Prescribed pressure derivative"
          annotation (Placement(transformation(extent={{-140,20},{-100,60}}, rotation=
                   0)));
          annotation (Diagram(graphics));
        end GetInputs;

      equation
          der(portA.p) = -simPort.dpdt;
          1 = -simPort.dpdtCounter;
          connect(sim.dpdtPort[pressureStateID], simPort.dpdtPort);

        connect(portB, portA) annotation (Line(
            points={{-60,0},{60,0}},
            color={153,204,0},
            thickness=0.5,
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-60,-60},{60,60}},
              initialScale=0.1), graphics={
              Bitmap(extent={{-60,-60},{60,60}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/PressureState.png"),
              Text(
                extent={{-100,-60},{100,-100}},
                lineColor={0,0,0},
                textString=
                     "%pressureStateType"),
              Text(
                extent={{-40,20},{90,-20}},
                lineColor={153,204,0},
                textString=
                     "(%pressureStateID)")}),Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-60,-100},{60,60}},
              initialScale=0.1), graphics),
              Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>differential states:</td><td>p</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>isobaric</td>
        </tr>
        </table>
          <p>
          The pressure state model is needed whenever a VLE fluid model with a volume is used.
          The mathematical description of the VLE fluid components is based on one major assumption:
          The term dp/dt (time derivative of pressure) in the balance equations is treated as constant along the direction of flow for each pressure level of the system.
          Note that the time derivative of the pressure is not constant over time.
          The pressure state model defines an initial pressure and a pressure level in the system during the simulation.
          The pressureStateID defines which volumetric model is connected to which pressure state level.
          </p>
          <hr>
          </html>"));
      end PressureState;

      package Internals
        extends TIL.Internals.ClassTypes.InternalPackage;

        type PressureStateType
        extends String;
         annotation(choices(
            choice="der(p)" "der(p) = dp/dt",
            choice="p=const" "pressure is constant",
            choice="p=f(t)" "p is given by input connector",
            choice="der(p)=f(t)" "der(p) is given by input connector"));

        end PressureStateType;
      end Internals;
    annotation(classOrder={"*","Internals"});
    end PressureStateElements;

    package Sensors "Sensors"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model Sensor_p "Sensor reads pressure p [Pa]"
        extends TIL.VLEFluidComponents.Sensors.BaseClasses.PartialSensor;

      /******************** Parameters ********************************/

        parameter SI.Pressure initialSensorValue = 0
          annotation(Dialog(enable=(useTimeConstant)));

      initial equation

      if useTimeConstant then
        sensorValue = initialSensorValue;
      end if;

      equation
        sensorValue_ = port.p;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                  {40,40}}),
                         graphics={Bitmap(extent={{-30,-40},{30,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAADwAAAA8CAYAAAA6/NlyAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA8RJREFUeNrkW+2xqjAQReb+1w6gA+1AO6AEtQLtADvgdoBWoK8CsQK1Au0ArYCXw4ATMQkBkgDXM7MzjijsySab/QgDSx/GRGZEJkTc7LMIEZE7kUv2+Wr1AB6RkEhMJGkocXYvr2skh0R8IjcFJHlyy54xbJusr8iaVazuN1F4UPN/UyLbbG2KfzidWrPZzHJdNxUW7vd7KlEUWafTSeb5WOsLIicT0zcQWWE4HCbz+TzZ7/dJXeC/uAfuVWLxQOc0d4iceQ93HCcJwzBRDdwT9xaQPme6Kd9iYp5FdRBlERdYPM501EvW87wkjuPEFPAsPFMnaSZZU1atYe1GpB0e2fP5nLQN6CAg7dTxxh8Oajwed4IsTRo6cRxZJe8ddNWyFSwdVAkqekFWgvRUhvBHTNymg6riyDgxeGls/LH19AWcLcsXOaq4OJVN7rMq9mnG1I55Dszv41SWnNp+6dpF/NpXMGLv11r+oSoVb7nbZrOpHZ7l6V4OpIfA4/GwDodDmgbm15Ey4vpisVAW+EP35XJJf+VmHP/lX4TFtdsEvu+/jTBwPB6F6R6uNUkpJdZySI/Am7NCLqqSMIjIVjXwXxUAB4bzeiUIHwqqJEyPNq7dbreXJYIg+LCGCmfJGeQ0sVgVLzRFkXAZkWKkpGo7ZOiwsrO68VsNSgeIJbmOaTKZpM4sx/P5tLbbbeNnMrhM7KJ3zj2q0tqQ41jr9Vr4GzyXVlAFYQYX1y52BHiVxSaQ3XLo312v17etrQ4YXGa2xI90jLSUghoIW7aJaj3WaJ2BQYCiGkYIj0ajzvSFjEzpLsFmxcFfRVgHZAfxcrnUWvuds3CRiCnCLC521m3XSpiOokSgvTKClab+hMElsrPWo9atAISRC5cpt9vtKgcrsgNItVnNJA8kbBTmr3RBHclDnlHpSB6iulOwCtDoxposTjNYAQEHQskciLubTmcOh8hIAaCY78KasDir79v02TIFAO0lHkFn4E1Wq5WxEo+nskTLqmlBCd4xBnyvspXDKdV62sq0LMI0UNDLxXSZVkshvoxwVwrxylotbRGWabXQoeWTyC89AqgtqSyQ6wZ0hc4F/Gbc9LVL27Bw3Xapkoa4acJVGuKs9PDEmtqIhmSzHpOATtCNM5WljyY2OtSC6YVIKhedluWc0qt8qCXN0KwvOrZE952+5mCakLT1R48elpL+i4dL6TVdenxYpcVxr7aOD9Peu9cHxFt7BQDdiLzO1eVXAFgZVq9e8lA1zb/mNZ4iOv+i1kAj+U6+ivdfgAEAjzCWGzLVhjQAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Sensor_p.png"),
              Text(
                extent={{-100,80},{100,40}},
                lineColor={0,0,0},
                textString=DynamicSelect("", String(sensorValue/1e5, significantDigits=4) + " bar"))}));
      end Sensor_p;

      model Sensor_subcooling "Sensor reads subcooling [K]"
        extends VLEFluidComponents.Sensors.BaseClasses.PartialSensor;
        TILMedia.VLEFluid_ph vleFluid(
          final vleFluidType=vleFluidType,
          final p=port.p,
          final h=inStream(port.h_outflow),
          final xi=inStream(port.xi_outflow)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}}, rotation=0)));

        parameter SI.TemperatureDifference initialSensorValue = 0
          annotation(Dialog(enable=(useTimeConstant)));

      initial equation

      if useTimeConstant then
        sensorValue = initialSensorValue;
      end if;

      equation

       sensorValue_ = smooth(0, if vleFluid.VLE.T_l - vleFluid.T > 0 then vleFluid.VLE.T_l - vleFluid.T else 0);

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                  {40,40}}),graphics={
                Bitmap(
                  extent={{-30,-40},{30,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAADwAAAA8CAYAAAA6/NlyAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAABJZJREFUeNrsWz1O60AQdnKBBC4QiwsQCXqngBoaaHkVtHRQUkLnFhpcQ8EBQCLpQXABkLkA+Ab75os2aL3eX8dOMGakUZTEnp1v52dnx+sOYyyogzqdzpA+RsT4DIkjyy0T4pT4lXhMer3WohgAV8VEu8QJcQbRc3LGZe1WqmMFIPvEZ9w6rCZO+Rj9pQLmSmQ1AlVZ/WwenTtlYpjic8TdbWC7NoqiYDQaBWEYTllFaZpOeTweB5PJxEWFD+J/pPu41hjm7hubrNDr9djBwQG7u7tjZQn3QgZkWSwe+7q5D9iQZ1Dl4IPBgF1fX7OqCTIh2wAaOoWVAuZLS6azaB1AVcANFoduw0oAm8Du7Oywr68vtijCWBhzHtClwC7KqiWsbQVti1kl2JeXF7Zsgg4G0KEXYJ6NCwlqfX39R4AVQUMnTSLr+wCOf6plPSwdOwHmBX8jwDqAHrkALtTEy0xQPolMVYMbAfPauLD0NIU0S9aZEjBPVJnsyotcZ6tYpxWunYkJrCuU1cfEPbHOjuM46Pf7QVMIukJniXocW37zIMcu6temkqL2TnMuzTsVjUtUnglsVwScyLHrQjc3N+z8/FxZ2+L3y8vLUgrj3r29vYJM/OYiUxPLiQg4l6ywFzXR09MT29jYcOpSrK2tsfv7eyegAOMqEzqYCBjk5DVrdgxlgabN+9vbG1tZWfFuz8AbTHR4eOgt0wQaGBT3DGfZOfeHiba2tnLXQlF5YHyXAcAqJheWdcD9mNwZwUvksSHz8/NTK1cB+LgQv1EUaQVAuHjtycmJNRZtVgYoWTFTCMixjTF0BCxyHHf5NvCb0HDT0fPzc+47zbhxXSQrBeT+2vtBt7e3ue8EwCgX/5vut2CZYnVejuCqctacl8Tkh9xgclGdlXX3aJan/A+Pj4/GweSEBYXhVi6K2kKkigkUCVhkfF3f8o3ituDmp6enwerqKvrVwcXFxZRdSHZxmrzay89SgBGbOgJ4MMBvb28bwb+/vy+83i4A1j0dEIkKhIAyrjVpPTw8fIO/urqyyhUT3MIA45GHC1G8BbR8TCs1ZE6T1UFHR0dWV6eScCFW9kpaNsK6iiQmFwmqjIr11nVNrS1puVpYR1T9TOMc1idwBbeHm4vXmpKYyRMQJjPe39/XPqRTufTEFTAEzwbZ3Nx0ikm5UBATFQCLcYvJcHFrcdJM2V2BZdLlG/9vwiNLHYmCYQ15YBeSE5MY+wDrktzka5BPVKTAknptHhB/YuGBosNWcMhVkbghKFNLy/U5Nhm+mwev7aE8ICZAlWywUVDtrFx3S9iYiJODslaePIwtT6B1e1imAaDLwCa2eUOZ/bDJE7QNgLItHlULRse41qXWrqrj4dLiKdXEw6BwR1W7Z7ap0LlcmZ4WvMClp2Vt4rWpTSsWHknumMzHR5AkSdA0gs7QXf5Z1YhvxaOW9j5Ma+Xj0lY+EG/dkYemHWrRnNLzO9TSumNLrI0H01gbjx6yNh4uZZ7Hh6u0OGQt5fgw+0UHxJf2CgBO3GRZ9rNfAWC/4CWPv9d42C9/UatUDDf5Vbz/AgwAy8IslpJh+D0AAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Sensor_sc.png"),
                Text(
                  extent={{-100,80},{100,40}},
                  lineColor={0,0,0},
                  textString=DynamicSelect("", String(sensorValue, significantDigits=4) + " K"))}));
      end Sensor_subcooling;

      model Sensor_superheating "Sensor reads superheating [K]"
        extends VLEFluidComponents.Sensors.BaseClasses.PartialSensor;
        TILMedia.VLEFluid_ph vleFluid(
          final vleFluidType=vleFluidType,
          final p=port.p,
          final h=inStream(port.h_outflow),
          final xi=inStream(port.xi_outflow)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}}, rotation=0)));

        parameter SI.TemperatureDifference initialSensorValue = 0
          annotation(Dialog(enable=(useTimeConstant)));

      initial equation

      if useTimeConstant then
        sensorValue = initialSensorValue;
      end if;

      equation
        sensorValue_ = smooth(0, if vleFluid.T - vleFluid.VLE.T_v > 0 then vleFluid.T - vleFluid.VLE.T_v else 0);

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                  {40,40}}), graphics={
                Text(
                  extent={{-100,80},{100,40}},
                  lineColor={0,0,0},
                  textString=DynamicSelect("", String(sensorValue, significantDigits=4) + " K")),
                Bitmap(
                  extent={{-30,-40},{30,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAADwAAAA8CAYAAAA6/NlyAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAABO1JREFUeNrsWz1S6zAQVjyvT+ACycAByAz0SQE1aaAlFZTQQZkSurTQkBoK0kOR9DBwARhzAQgn8NNnZD95s5Z/YjtO8r4ZTUC25f20q9VKWlccxxF5oFKpNOVPWxb8NmRpRTwylsWW5U2WkZTrLRfBQDirItGRZSDLBE3PWCaqrU6mMmZAsiZLT2nHyanY6h21uRJWQkxyJMppvTeLzJU0Y1iOz7Yyt3rUva1WS7TbbdFoNNzCwbZtt4xGIzEej+OI8ClLV8o+ynUMK/Ptm7RQrVado6Mj5+HhwUkLPIs20FaExvtJzTwJ2YbyoOzL6/W6c3t762QNtIm2DaQhUyNTwmpqmYRpNA+iHHGDxiFbMxPCJrL7+/vO9/e3UxTwLrxzFtKpyBal1RTajiQdNWZZsq+vr868ARkMpBuJCCtvPOWgtra2SkFWJw2ZQhxZLQnhfhGavb6+Drzj8fExS033OW5WSFBxqtfJBt2goNlsirIBMkE2yEhwqrgEYDFtDGhFv98vJVmdNGSMw8Ui2u3RcFFOA6Lb7YqyAzJCVoK64jRNWF6AozqjpjwYDMSiALIypn2muE1pGGSr1JRrtdrCEIasjGlXdUXqhAN2K+PXhTBlzrQhO60OEJYq79Cx2+v1xKKCkb2uOPoa7tCxG6Xd+/t7cXV1hc6aKqi/ublJLbCMmd02Njc3A+3u7Oy49bhuQqfT4cZyx18P0xASa9EwPD8/O9vb27F2KDY2NozBBBd40Lqwcnd3ZwxIwIGGnN5mR5M2FrZ4f39/d9bW1hJvzYQJR8nt7u5m0q63icA80/S8c+BCGKhAx8fHrsapBaCeajoOYf3+y8vLqXtRH6ddD0zbZ1404le2Wi324a+vr8DD5+fnxpdB4ChtcITRqXhXmAyUNO1wHeBC2h9YahnoAxtuHF5eXgL/S8GMjkNqWUjzD32eA+6XHRN4jl5Huya5Irg0/tATgbCdRSoEvLCJNO6XGknknWkncaDvNHlshkvLinGTC+mZA8JgWoo7TcSFNNdYVhAXHBcriUBy3E6Z08XFhVhfX/fnX5Q8Cc+KxITpGNIB8iggv7e3NxP5wgiHmbQH6VldxxLltJ6ennzys0RduRPGkUcUDg4OhIyK3ChNTj9GrQMnJyel0bY1awMwc2gd5GUk5nYAp31oOyvnVriGTU4HHQDtY0qixGHmRYLjYqmT90jCh4eHgVVLnOkD2tbx8fExb8JjSx02+8AOYNg8rE9HabSVZA7NAgwX21Kb1v+6IOR8lkZBccYkdVRRnj1rMFzeQHiqG4bDIasdPfCAlrFA57wvojDMw/jVO6yIwMLEweWadAMg6ZoVBRsG3AoozckD1uT6M3QZGbUB4HnpYYzecQEPjHk4Lrw5u8jxO5lMOA5DfVoKXP35+THuRyPSkutQ1wvrzkx3cLiGedm03MvTnMGBI6wfoNmCpDAsKpgUCZs7TAuo9PPzc6FOHfTTB8hOq/2/yJlwwHnhGLLIlIYsUiKYo1NwqrHnw+I30Wwqj2NREJL/0TMeiAsmhXCe+RxJ8j4YsnZkBoD4zYAtZV5HiiyAdqlSHnIm21/KpJaQLL1kSS0rl7a0kolpK5l6uJLJpUnTh7PUONqaS/rwMiWIz+0TAGTcYN3qbbaV8hOAZfjI4/9nPMv+oVaqMRxznJfyU7y/AgwA9RC49nmL5UsAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Sensor_sh.png")}));
      end Sensor_superheating;

      model Sensor_T "Sensor reads temperature T [K]"
        extends TIL.VLEFluidComponents.Sensors.BaseClasses.PartialSensor;

      /******************** VLEFluid ********************************/
      protected
        TILMedia.VLEFluid_ph vleFluid(final p = port.p, final h = inStream(port.h_outflow),
          final xi = inStream(port.xi_outflow), final vleFluidType = vleFluidType,
          computeTransportProperties=false) "VLEFluid"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));

      /******************** Parameters ********************************/

      public
        parameter SI.Temperature initialSensorValue = 0
          annotation(Dialog(enable=(useTimeConstant)));

      initial equation

      if useTimeConstant then
        sensorValue = initialSensorValue;
      end if;

      equation
        sensorValue_ = vleFluid.T;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                  {40,40}}),
                         graphics={
              Text(
                extent={{-100,80},{100,40}},
                lineColor={0,0,0},
                textString=DynamicSelect("", String(sensorValue, significantDigits=4) + " K")), Bitmap(
                extent={{-30,-40},{30,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAADwAAAA8CAIAAAC1nk4lAAAABnRSTlMA/wAAAACkwsAdAAAACXBIWXMAAAsTAAALEwEAmpwYAAADFElEQVR42u2a27WiMBSGY9a8kw6gA9IB6UA7ICXQgXQCVgAdnFgBmQ5iBdEKOA/OOHEDMVyEuM7s5ZuYfP7sXPZl16K59ltKIYSUUiklhOh9hjEWRRGllDEWUzp3ynbqp65rzjkhZOyMhBDOeV3Xk6ee8ps8z6Momv2GUBRFeZ6/HTrP8wnSvhR+LPrO0afPQnDOlVJDDyRJcnfc7ktQSt3d/Xw+W1QvyzJhbDGfzrKs97dBEKRpWlVV62xVVaVpGgRB74BZlkkptdbT3UNrrZSifYs9DMOiKNoZVhRFGIbdkSmldV3buW3EUsquBwdBMBMXoHdVJ4TYuccR7/d7rXW7qGmt9/v9KG5X4mUFdpHcwt0DrZTqEjdN077Zmqbp5XaCBisvjuMViB/ccRyDdfkaGuxu62hs1zvLMhs0uO6sTzzELYQYhAaH2VtX3st1Cc7Lfug8z8Hu1m5qYB807yf/oM0dIwiCxffjCfu36SSEEAgNZN7QMSxO8hC7x5vDMHQZcf6lNEmSl7OY95OHZ/+JQSbIvA40EPt+1qAWIc656c2O724daODZnPMWoV/oWenD4eA45fF4HPrKvO+naToUm7nEbISQw+FwOp0eShcIISml+dCoG/2Qmf/n6+tr5mhVVZmEUkoMTkF3pVczgCSEwKbSSZIgL80Ek1JiM1ZljnHl6maCKaWe3GORbMY7zAQTQuCxy3lzaIQQRh9onw/trXvYoC1Zr//u8TOUBmAY7Nv+QzPGMNi3/YQGJyA2UzOW/PG2ZoJRSjG4b4AoxgcDSIwxHFNqxuGeQxNCYkoxuLD6Bn29XruBFYS+3W5lWXol8+12gwHB5BTCOuFWbwoBP6Lcx3eXy8UTscuyvFwuZigOq1sLpsUWUdqSFsNmGtj0bFP7TYxzbnrzU+L8Hane+Uq7pnoXTKoXRZH8tQkjjEuqf2T5wodCESjjOhWKfCvJKaU+rPg5VNz3t8xsaUfwtKBvb6CY1ToxWXWttaV1Qik1sXViqyaVjduBCCHX6xVt1Q7kW+PVD2hx86SZcPeJbZvfgLuCmypm6lkAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Sensor_T.png")}));
      end Sensor_T;

      model StatePoint "State point"

      /*********************** SIM ***********************************/

        parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
          "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
          choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
          choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
          choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";

      /******************** Connector *****************************/
      public
          Connectors.VLEFluidPort sensorPort(final vleFluidType = vleFluidType)
          "Sensor port"
          annotation (Placement(transformation(extent={{-10,-30},{10,-10}}, rotation=
                  0), iconTransformation(extent={{-10,-50},{10,-30}})));

      /****************** Parameters ********************/
        parameter Integer stateViewerIndex=0 "Index for StateViewer" annotation(Dialog(group="StateViewer Index"));

      /***************** Variables *********************/
        SI.SpecificEnthalpy h "Specific enthalpy";
        SI.AbsolutePressure p "Pressure";

      equation
        h = inStream(sensorPort.h_outflow);
        p = sensorPort.p;
        sensorPort.m_flow = 0;
        sensorPort.h_outflow = 0;

        sensorPort.xi_outflow = zeros(vleFluidType.nc-1);

        sensorPort.h_limit = -1e6;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-40},
                  {40,40}}), graphics={            Text(
                extent={{-30,-14},{110,-60}},
                lineColor={153,204,0},
                textString=
                     "%stateViewerIndex"),
              Text(
                extent={{-100,100},{100,60}},
                lineColor={0,0,0},
                textString=DynamicSelect("", String(p/1e5, significantDigits=4) + " bar")),
              Text(
                extent={{-100,60},{100,20}},
                lineColor={0,0,0},
                textString=DynamicSelect("", String(h/1e3, significantDigits=4) + " kJ/kg")),
              Bitmap(
                extent={{-30,-40},{30,20}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAADwAAAA8CAIAAAC1nk4lAAAABnRSTlMA/wAAAACkwsAdAAAACXBIWXMAAAsTAAALEwEAmpwYAAAEdklEQVR42uWa3XWrOBCAic++Sx1AB6gD6MB0gFIBdGBcgenAdGDdCoAK0FZguQKgAvZBu8pECBkwdpyzc/IQ2/r5GGZGo5E+BudR+Zvzqqo450KIqqqMbcIw9DyPEBKGoU/Io1MOa/8YY5RSjPHSGTHGlFLG2Oqp1/TJsszzvIffkON5XpZlT4fOsmyFau8qfin6x0ybrquKUiqEmGoQBIE03PFLEEJIc6/r2qL1oiiCMNzMptM0NfZFCMVxfLlchtlyuVziOEYIGQdM03QD8xBCEJOzu657Pp+HB+R8PruuOx6ZEHLXR22/cc7HFowQehBXQx9rHWNs515GvN/v27YdNpW2bff7/SLuucTbKniOyi3cZjseEzdNMzxZmqaZyW2A1jzP9/0XECtu3/c1v7wPrUW3x3VclqUarSzLFfoex8FvH7R0ZxOrWApt5K6qahJaW8w28bwV0NIvtfXSDJ1lmRbdNjHTddDDMGhxEOYnX9AwYiCEtorHq6HbtoVGgjFWqDv51THLuq5TLfI83zybW5H95XmuPnZdd1S2MLZm13U3jGKrNS0F5ifKsv9yHOcPYyKKoPWo/znn8g1gjGX8FkIwxqqqkt8TQqIoCmemlCAaSFGbsSiKjJlZlmWfn58qxf3D2D6KnMFxKKXQmuGDBkGg0uVhGE6nkxHCsgBpmm6aRo05zsiv16vdsiml/zoiNN84jqegkySxKA8hZEysIfTpdJrKpC0BII5jzR0dzjnspk2soNVkMnOSQ1+vV/gkxsUIQkOlns/nsizLskySBD6JpjW5b4B9OecO9FDHcbQO2qs0YsGFQFqRBRohNHZHbQkcKxuOkOf5N4MeT6lBTxnu4XCYaqNBTwUQOMK4DcSglO7gXtUeBOI4JhN1ljRNlaqKopgawff9qSkiEL7GFR/YSwixgy3s1Qz4TsYLgZqVMTbVDJKNt4b2Cgl8pN3Ub8bS1pxxb7fbOrKZ0I5axu+KlpvbH2mqordVajAX+sdTkUnoTSp0r4a2VL3eF9oid58Hrqzk8Qr0Jpq+3W4w4bZAu667uQNoYDstblt6WgIwjBhL09Sl0GEY7rS4belpWeqKolDh2bIGrRZtBdxB+7PUj+Wv2uZXGYaqllgW6kcEghFCdtocdhs4Ho+UUmjcRVGEYdj3/VcKtrVoSGEYzt0EaMl7EARBEGhfGusk8/eIqtnhcLBvAnZaHjOlaUIIzJvruq7rWilY7gyeYc1d10EkiapD930/5XCU0qZptBoKQihJEiHEFDHGOPhP7KFQNYOxgTGmVPOFai8haBtbtdmUO6UXVFONJQRzTUxZpxH6ZaJV9FRl7E5Z7Aeh75fFnO+V6b7vn+FVi4RSCq35W+HcXur9KU3PLfUai+rKD14JvayobjmcfRn04uML40GRDIJJkryGWDvGnXVQ9G5HckKIX3b4yTn/ZcfMU8Tve6BvIX706sRqrbdta7k6YbTjxdeB3u2SyhOvA2GM5R7nZ64DvdvFq//BFbc3uUz48Ruvbf4D0RN8GER0TmUAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Sensor_ph.png")}),
              Documentation(info="<html>
          <p>
          The State Point reads pressure and enthalpy.
          The state points for a p,h-diagram in the StateViewer can be automatically detected.
          </p>
          <hr>
          </html>"));
      end StatePoint;

      package BaseClasses
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialSensor

        /*********************** SIM ***********************************/

          parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
            "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
            choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
            choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
            choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));
        protected
          outer SystemInformationManager sim "System information manager";

        /******************** Connector *****************************/
        public
          TIL.Connectors.VLEFluidPort port(final vleFluidType = vleFluidType) "Port"
            annotation (Placement(transformation(
                origin={0,-20},
                extent={{-10,-10},{10,10}},
                rotation=180), iconTransformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={0,-40})));

          Modelica.Blocks.Interfaces.RealOutput sensorValue "Sensor value"
            annotation (Placement(transformation(
                origin={0,42},
                extent={{-10,-10},{10,10}},
                rotation=90), iconTransformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,22})));

        /******************** Time Constant ************************/

          parameter Boolean useTimeConstant = false
            "= true, if time constant tau is used";

          parameter Modelica.SIunits.Time tau = 1
            "Time constant for delay time of the sensor value"
            annotation(Dialog(enable=(useTimeConstant)));

          Real sensorValue_;

        equation
           if useTimeConstant then
             der(sensorValue) = (sensorValue_ - sensorValue)/tau;
           else
             sensorValue = sensorValue_;
           end if;

          port.m_flow = 0;
          port.h_outflow = 0;

          port.xi_outflow = zeros(vleFluidType.nc-1);

          port.h_limit=-1e6; //no cell volume!

          annotation (Diagram(graphics),
                               Icon(coordinateSystem(preserveAspectRatio=true, extent={
                    {-40,-40},{40,40}}),
                                    graphics={
                Text(visible=useTimeConstant,
                  extent={{10,50},{50,10}},
                  lineColor={0,0,0},
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid,
                  fontName="Symbol",
                  textString="t")}));
        end PartialSensor;
      end BaseClasses;
    annotation(classOrder={"*","Internals","BaseClasses","Testers"});
    end Sensors;

    package Separators "Separators"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model IdealSeparator "Ideal separator model"

      /*********************** SIM ***********************************/

        parameter TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType = sim.vleFluidType1
          "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
          choice=sim.vleFluidType1 "VLE Fluid 1 as defined in SIM",
          choice=sim.vleFluidType2 "VLE Fluid 2 as defined in SIM",
          choice=sim.vleFluidType3 "VLE Fluid 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";
        TIL.Internals.SimPort simPort;
        parameter Boolean removeSingularity=sim.removeSingularity annotation(Evaluate=true);
        parameter Boolean generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal  annotation(Evaluate=true);
      public
        parameter TIL.Internals.PressureStateID pressureStateID=1 "Pressure state ID"
          annotation(Dialog(tab="SIM",group="SIM"));

      /*********************** Connectors ***********************************/

        TIL.Connectors.VLEFluidPort portInlet(final vleFluidType=vleFluidType, m_flow(start=mdotStart)) "Inlet"
          annotation (Placement(transformation(extent={{-60,50},{-40,70}}, rotation=0),
              iconTransformation(extent={{-60,30},{-40,50}})));
        TIL.Connectors.VLEFluidPort portGas(final vleFluidType=vleFluidType, m_flow(start=-mdotStart))
          "Outlet with pure gaseous conditions"
          annotation (Placement(transformation(extent={{40,50},{60,70}}, rotation=0),
              iconTransformation(extent={{40,30},{60,50}})));
        TIL.Connectors.VLEFluidPort portLiquid(final vleFluidType=vleFluidType, m_flow(start=-mdotStart))
          "Outlet with pure liquid conditions"
          annotation (Placement(transformation(extent={{-10,-90},{10,-70}}, rotation=0),
              iconTransformation(extent={{-10,-110},{10,-90}})));

       parameter Boolean enableHeatPort = false "true, if heat port is enabled"
                                           annotation(Dialog(group = "General"));
        TIL.Connectors.HeatPort heatPort if enableHeatPort annotation (Placement(
              transformation(extent={{-60,10},{-40,30}}, rotation=0),
              iconTransformation(extent={{-60,-10},{-40,10}})));

      /*********************** VLE Fluids ***********************************/

      protected
        TILMedia.VLEFluid_ph
          vleFluidPortInlet(
            final vleFluidType = vleFluidType,
            final p=portInlet.p,
            final h=noEvent(actualStream(portInlet.h_outflow)),
            final xi=noEvent(actualStream(portInlet.xi_outflow))) if includeDefaultSummary
          "VLEFluid at port inlet"
              annotation (Placement(transformation(extent={{-60,80},{-40,100}}, rotation=0)));
        TILMedia.VLEFluid_ph
          vleFluidPortLiquid(
            final vleFluidType = vleFluidType,
            final p=portLiquid.p,
            final h=noEvent(actualStream(portLiquid.h_outflow)),
            final xi=noEvent(actualStream(portLiquid.xi_outflow))) if
                 includeDefaultSummary "VLEFluid at liquid outlet"
              annotation (Placement(transformation(extent={{-10,-60},{10,-40}}, rotation=0)));
        TILMedia.VLEFluid_ph
          vleFluidPortGas(
            final vleFluidType = vleFluidType,
            final p=portGas.p,
            final h=noEvent(actualStream(portGas.h_outflow)),
            final xi=noEvent(actualStream(portGas.xi_outflow))) if includeDefaultSummary
          "VLEFluid at gas outlet"
              annotation (Placement(transformation(extent={{40,80},{60,100}}, rotation=0)));

        TILMedia.VLEFluid_ph
          vleFluid(final p=p, final h=h, xi=xi, final vleFluidType=vleFluidType)
          "VLEFluid representing state of inner volume"
            annotation (Placement(transformation(extent={{-10,40},{10,60}}, rotation=0)));

      /**********************************************************/

      public
        parameter SI.Volume V=500e-6 "Volume"
          annotation(Dialog(group="Geometry"));

        parameter Real yLimit=0.95 "Singularity removal coefficient. The greater yLimit, the lesser the influence on h_outflow.
    Should be > 0.9 and < 1 to prevent singularity."                                                          annotation(Dialog(tab="Advanced"));

        parameter SI.SpecificEnthalpy delta_h = 10
          "Distance of outlet enthalpy to VLE curve"                                          annotation(Dialog(tab = "Advanced"));

      protected
        parameter TIL.Internals.CellFlowType cellFlowType = "allow reverse flow";

        Real extendedFillingLevel
          "Filling level (>1 if superheated , <0 if subcooled)";
      public
        Real fillingLevel "Filling Level";

      /**************** Start and Initialization *********************/

      public
        parameter Real initialFillingLevel=0.5 "Initial filling level"
          annotation(Dialog(group="Initial values"));
        parameter Real[vleFluidType.nc] mixingRatioInitial=vleFluidType.defaultMixingRatio
          "Initial value for mass fraction" annotation(Dialog(group="Initial values"));

        parameter SI.MassFlowRate mdotStart = 0.1
          "Start value for inlet and outlet massflowrate" annotation(dialog(tab="Start Values"));
        parameter SI.SpecificEnthalpy hStart = 350e3
          "Start value for specific enthalpy " annotation(dialog(tab="Start Values"));

      public
        SI.AbsolutePressure p "Pressure";
        SI.SpecificEnthalpy h(final start=hStart) "Specific enthalpy";
        SI.Temperature T;
        SI.Mass mass "Mass";
        SI.MassFraction[vleFluidType.nc-1] xi "Mass fraction";
        SI.SpecificEnthalpy hGasOutlet "Outlet specific enthalpy";
        SI.SpecificEnthalpy hLiquidOutlet "Outlet specific enthalpy";
        SI.MassFraction[vleFluidType.nc-1] xiGasOutlet "Outlet mass fraction";
        SI.MassFraction[vleFluidType.nc-1] xiLiquidOutlet "Outlet mass fraction";

      protected
        Real wfliq "Weighting factor for liquid outlet";
        Real wfgas "Weighting factor for gas outlet";
        Real drhodt "Time derivative of density";

        Internals.GetInputsThermal getInputsThermal annotation (Placement(transformation(extent={{-10,10},{10,30}}, rotation=0)));

      /*********************** Summary ***********************************/

      public
        parameter Boolean includeDefaultSummary = true
          "Include summary record in model results"
       annotation(Dialog(tab="Advanced", group="Summary"));

      protected
        record Summary
          extends TIL.Internals.ClassTypes.Record;

          SI.Pressure p_Inlet "Pressure at port inlet";
          SI.Pressure p_Gas "Pressure at port gas";
          SI.Pressure p_Liquid "Pressure at port liquid";
          SI.Temperature T_Inlet "Temperature at port inlet";
          SI.Temperature T_Gas "Temperature at port gas";
          SI.Temperature T_Liquid "Temperature at port liquid";
          SI.SpecificEnthalpy h_Inlet "Specific enthalpy at port inlet";
          SI.SpecificEnthalpy h_Gas "Specific enthalpy at port gas";
          SI.SpecificEnthalpy h_Liquid "Specific enthalpy at port liquid";
          SI.Density d_Inlet "Density at port inlet";
          SI.Density d_Gas "Density at port gas";
          SI.Density d_Liquid "Density at port liquid";
          SI.MassFlowRate m_flow_Inlet "Mass flow rate at port inlet";
          SI.MassFlowRate m_flow_Gas "Mass flow rate at port gas";
          SI.MassFlowRate m_flow_Liquid "Mass flow rate at port liquid";
          SI.TemperatureDifference superheating "Superheating at port gas";
          SI.TemperatureDifference subcooling "Subcooling at port liquid";
          SI.MassFraction q_Inlet "Vapor quality (steam mass fraction) at port inlet";
          SI.MassFraction q_Gas "Vapor quality (steam mass fraction) at port gas";
          SI.MassFraction q_Liquid
            "Vapor quality (steam mass fraction) at port liquid";
          Real fillingLevel "Volume fraction";
          SI.Mass mass "Total fluid mass";
          SI.Volume volume "Total separator volume";

        end Summary;

        replaceable record SummaryClass = Summary;

      public
        SummaryClass summary(
          p_Inlet = portInlet.p,
          p_Gas = portGas.p,
          p_Liquid = portLiquid.p,
          T_Inlet = vleFluidPortInlet.T,
          T_Gas = vleFluidPortGas.T,
          T_Liquid = vleFluidPortLiquid.T,
          h_Inlet = vleFluidPortInlet.h,
          h_Gas = vleFluidPortGas.h,
          h_Liquid = vleFluidPortLiquid.h,
          d_Inlet = vleFluidPortInlet.d,
          d_Gas = vleFluidPortGas.d,
          d_Liquid = vleFluidPortLiquid.d,
          m_flow_Inlet = portInlet.m_flow,
          m_flow_Gas = portGas.m_flow,
          m_flow_Liquid = portLiquid.m_flow,
          q_Inlet = vleFluidPortInlet.q,
          q_Gas = vleFluidPortGas.q,
          q_Liquid = vleFluidPortLiquid.q,
          superheating = noEvent(max(0, max(vleFluidPortGas.T -vleFluidPortGas.VLE.T_v,
            vleFluidPortInlet.T -vleFluidPortInlet.VLE.T_v))),
          subcooling = noEvent(max(0, max(vleFluidPortLiquid.VLE.T_v- vleFluidPortLiquid.T,
            vleFluidPortInlet.VLE.T_v - vleFluidPortInlet.T))),
          fillingLevel = fillingLevel,
          mass = mass,
          volume = V) if includeDefaultSummary
          annotation (Placement(transformation(extent={{40,-80},{60,-60}}, rotation=0)));

      initial equation
      // equation valid for ideal mixture of liquid and vapor
        vleFluid.h=(vleFluid.VLE.h_v + 1 - initialFillingLevel*(vleFluid.VLE.h_v + 1 - vleFluid.VLE.h_l*vleFluid.VLE.d_l/vleFluid.VLE.d_v))/(initialFillingLevel*(vleFluid.VLE.d_l/vleFluid.VLE.d_v - 1) + 1);

        xi = mixingRatioInitial[1:end-1]/sum(mixingRatioInitial);

      equation
        assert(sim.dpdtPort[pressureStateID].counter>0.5,"Pressure state ID is not valid");
        connect(sim.fluidPort[vleFluidType.ID],simPort.vleFluidPort);
        simPort.vleFluidMass=mass;
        simPort.vleFluidVolume=V;

        portInlet.p - portGas.p = 0 "momentum balance";
        portInlet.p - portLiquid.p = 0 "momentum balance";

        p = portInlet.p;

        if (removeSingularity) then
          portInlet.h_outflow -h = noEvent(max(0,inStream(portInlet.h_limit)-h));
          portGas.h_outflow -hGasOutlet = noEvent(max(0,inStream(portGas.h_limit)-hGasOutlet));
          portLiquid.h_outflow -hLiquidOutlet = noEvent(max(0,inStream(portLiquid.h_limit)-hLiquidOutlet));
        else
          portInlet.h_outflow -h = 0;
          portGas.h_outflow -hGasOutlet = 0;
          portLiquid.h_outflow -hLiquidOutlet = 0;
        end if;

        portInlet.h_limit =vleFluid.h+yLimit*vleFluid.d/vleFluid.drhodh_pxi;
        portGas.h_limit = vleFluid.h+yLimit*vleFluid.d/vleFluid.drhodh_pxi;
        portLiquid.h_limit = vleFluid.h+yLimit*vleFluid.d/vleFluid.drhodh_pxi;

        portInlet.xi_outflow = xi;
        portGas.xi_outflow = xiGasOutlet;
        portLiquid.xi_outflow = xiLiquidOutlet;

        T = vleFluid.T;

        drhodt = vleFluid.drhodh_pxi*der(h)
               + vleFluid.drhodp_hxi*sim.dpdtPort[pressureStateID].dpdt
               + vleFluid.drhodxi_ph*der(xi);

        portInlet.m_flow + portGas.m_flow + portLiquid.m_flow= V*drhodt
          "mass balance";

        mass = V*vleFluid.d;

        getInputsThermal.heatPort.T = vleFluid.T;

        if (generateEventsAtFlowReversal) then
          der(h) = 1/mass*(portInlet.m_flow*(actualStream(portInlet.h_outflow)-h)
                           + portGas.m_flow*(actualStream(portGas.h_outflow)-h)
                        + portLiquid.m_flow*(actualStream(portLiquid.h_outflow)-h)
                        + V*sim.dpdtPort[pressureStateID].dpdt + getInputsThermal.heatPort.Q_flow)
            "energy balance";
          der(xi) = 1/mass*(portInlet.m_flow*(actualStream(portInlet.xi_outflow)-xi)
                            + portGas.m_flow*(actualStream(portGas.xi_outflow)-xi)
                         + portLiquid.m_flow*(actualStream(portLiquid.xi_outflow)-xi))
            "mass balance";
        else
          der(h) = 1/mass*noEvent(portInlet.m_flow*(actualStream(portInlet.h_outflow)-h)
                                  + portGas.m_flow*(actualStream(portGas.h_outflow)-h)
                               + portLiquid.m_flow*(actualStream(portLiquid.h_outflow)-h)
                               + V*sim.dpdtPort[pressureStateID].dpdt + getInputsThermal.heatPort.Q_flow)
            "energy balance";
          der(xi) = 1/mass*noEvent(portInlet.m_flow*(actualStream(portInlet.xi_outflow)-xi)
                                   + portGas.m_flow*(actualStream(portGas.xi_outflow)-xi)
                                + portLiquid.m_flow*(actualStream(portLiquid.xi_outflow)-xi))
            "mass balance";
        end if;

        noEvent(max(vleFluid.VLE.h_v-vleFluid.VLE.h_l,1e-12))*extendedFillingLevel = (vleFluid.VLE.h_v - h)*vleFluid.d/vleFluid.VLE.d_l;
        wfliq=TIL.Utilities.Numerics.smoothTransition(extendedFillingLevel, 0.05, 0.1);
        wfgas=TIL.Utilities.Numerics.smoothTransition(-extendedFillingLevel, -0.95, 0.1);
        hGasOutlet = if (noEvent(vleFluid.h>vleFluid.VLE.h_v+10)) then vleFluid.h else (vleFluid.VLE.h_v + delta_h)*(1-wfgas)+(wfgas)*vleFluid.h;
        hLiquidOutlet = if (noEvent(vleFluid.h<vleFluid.VLE.h_l-10)) then vleFluid.h else (vleFluid.VLE.h_l - delta_h)*(1-wfliq) + (wfliq)*vleFluid.h;
        xiGasOutlet = if (noEvent(vleFluid.h>vleFluid.VLE.h_v+10)) then vleFluid.xi else (vleFluid.VLE.xi_v)*(1-wfgas)+(wfgas)*vleFluid.xi;
        xiLiquidOutlet = if (noEvent(vleFluid.h<vleFluid.VLE.h_l-10)) then vleFluid.xi else (vleFluid.VLE.xi_l)*(1-wfliq) + (wfliq)*vleFluid.xi;
        fillingLevel = noEvent(max(noEvent(min(extendedFillingLevel,1)),0));
        // +-10 to make shure that the vleFluid object is not inside vapor region

        connect(heatPort, getInputsThermal.heatPort) annotation (Line(
            points={{-50,20},{-10,20}},
            color={204,0,0},
            thickness=0.5,
            smooth=Smooth.None));

        annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-60,
                  -80},{60,100}}),
                            graphics),
                             Icon(coordinateSystem(preserveAspectRatio=true, extent={{-60,
                  -100},{60,100}}),
                                  graphics={
              Line(points={{-40,-60},{40,-60}}, color={255,255,255}),
              Bitmap(extent={{-50,-102},{50,82}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/Seperator_VLE.png"),
              Text(
                extent={{-80,-43},{80,-83}},
                lineColor={153,204,0},
                textString=
                     "(%pressureStateID)")}),
              Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>h, xi, derivative of pressure in the PressureStateElement</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>isobaric</td>
        </tr>
        </table>
          <p>
          The ideal separator model separates the VLE fluid at the inlet port into the liquid and gas phase.
          If the filling level is between 5% and 95% the portGas and portLiquid provides pure gaseous respectively pure liquid conditions.
          If the filling level is lower than 5% the portLiquid provides a mixture of gaseous and liquid conditions according to the filling level.
          If the filling level is higher than 95% the portGas provides a mixture of gaseous and liquid conditions according to the filling level.
          The following picture illustrates three enthalpies dependent on the filling level:
          </p>
          <br><br>
          <img src=\"modelica://TIL/Images/IdealSeparatorOutletEnthalpy.png\" width=\"731\">
          <hr>
          </html>"));
      end IdealSeparator;
    annotation(classOrder={"*","Internals","Testers"});
    end Separators;

    package Valves "Valves"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model OrificeValve "Orifice valve"

      /*********************** SIM ***********************************/

        parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
          "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
          choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
          choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
          choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));
      protected
        outer SystemInformationManager sim "System information manager";

      public
        parameter Boolean use_effectiveFlowAreaInput = false
          "= true, if effectiveFlowArea defined by input";
        parameter SI.Area effectiveFlowAreaFixed = 0.2e-6 "Effective flow area"
          annotation(Dialog(enable=not use_effectiveFlowAreaInput));

      /******************** Connectors *****************************/

        Modelica.Blocks.Interfaces.RealInput effectiveFlowArea_in if use_effectiveFlowAreaInput
          "Prescribed effective flow area [m^2]"
          annotation (Placement(transformation(
              origin={0,50},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        TIL.Connectors.VLEFluidPort portA(final vleFluidType = vleFluidType) "portA"
          annotation (Placement(transformation(extent={{-90,-10},{-70,10}}, rotation=
                  0)));
        TIL.Connectors.VLEFluidPort portB(final vleFluidType = vleFluidType) "portB"
          annotation (Placement(transformation(extent={{70,-10},{90,10}}, rotation=0)));

      /******************** VLEFluid Objects ********************/

        TILMedia.VLEFluid_ph vleFluidA(
          p=portA.p, h=inStream(portA.h_outflow),
          xi = inStream(portA.xi_outflow),
          final vleFluidType = vleFluidType,
          computeTransportProperties=false)
          annotation (Placement(transformation(extent={{-100,20},{-80,40}}, rotation=0)));
        TILMedia.VLEFluid_ph vleFluidB(
          p=portB.p,
          h=inStream(portB.h_outflow),
          xi = inStream(portB.xi_outflow),
          final vleFluidType = vleFluidType,
          computeTransportProperties=false)
          annotation (Placement(transformation(extent={{80,20},{100,40}}, rotation=0)));
        TILMedia.VLEFluid_ph vleFluidOut(
          p=noEvent(min(portA.p,portB.p)),
          h=if noEvent(portA.p>portB.p) then inStream(portA.h_outflow) else inStream(portB.h_outflow),
          xi = if noEvent(portA.p>portB.p) then inStream(portA.xi_outflow) else inStream(portB.xi_outflow),
          final vleFluidType = vleFluidType,
          computeTransportProperties=false) if includeDefaultSummary
          annotation (Placement(transformation(extent={{20,20},{40,40}},  rotation=0)));

        final parameter Boolean generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal  annotation(Evaluate=true);

        parameter SI.MassFlowRate mdotSmooth = 0.0005
          "below this value, root function is approximated"
          annotation(Dialog(tab="Advanced",group="Smoothing Function"));
        parameter SI.Area effectiveFlowAreaTypical = 0.2e-6
          "Typical effective flow area"
          annotation(Dialog(tab="Advanced",group="Smoothing Function"));

        final parameter Real x0 = (mdotSmooth/effectiveFlowAreaTypical)^2;
        Real x;

      /*******************Summary**********************/
      public
        parameter Boolean includeDefaultSummary = true
          "Include summary record in model results"
       annotation(Dialog(tab="Advanced", group="Summary"));

      protected
        record Summary
          extends TIL.Internals.ClassTypes.Record;

          SI.Pressure p_A "Pressure at port A";
          SI.Pressure p_B "Pressure at port B";
          SI.Temperature T_A "Temperature at port A";
          SI.Temperature T_B "Temperature at port B";
          SI.Temp_C T_degC_A "Temperature at port A";
          SI.Temp_C T_degC_B "Temperature at port B";
          SI.SpecificEnthalpy h_A "Specific enthalpy at port A";
          SI.SpecificEnthalpy h_B "Specific enthalpy at port B";
          SI.Density d_A "Density at port A";
          SI.Density d_B "Density at port B";
          SI.MassFlowRate m_flow_A "Mass flow rate at port A";
          SI.MassFlowRate m_flow_B "Mass flow rate at port B";
          SI.TemperatureDifference superheating "Superheating";
          SI.TemperatureDifference subcooling "Subcooling";
          SI.MassFraction q_A "Steam mass fraction (quality) at port A";
          SI.MassFraction q_B "Steam mass fraction (quality) at port B";
          SI.Area effectiveFlowArea "Effective flow area";
          SI.Velocity w_A "Speed of sound at port A";
          SI.Velocity w_B "Speed of sound at port B";
          SI.Velocity velocity_A "Flow velocity at port A";
          SI.Velocity velocity_B "Flow velocity at port B";
          SI.Pressure dp "Total pressure drop";
          SI.Mass mass "Total fluid mass";
          SI.Volume volume "Total fluid volume";

        end Summary;

        replaceable record SummaryClass = Summary;

      public
        SummaryClass summary(
          p_A = vleFluidA.p,
          p_B = vleFluidB.p,
          T_A = if noEvent(portA.m_flow>0) then vleFluidA.T else vleFluidOut.T,
          T_B = if noEvent(portB.m_flow>0) then vleFluidB.T else vleFluidOut.T,
          T_degC_A = SI.Conversions.to_degC(summary.T_A),
          T_degC_B = SI.Conversions.to_degC(summary.T_B),
          h_A = if noEvent(portA.m_flow>0) then vleFluidA.h else vleFluidOut.h,
          h_B = if noEvent(portB.m_flow>0) then vleFluidB.h else vleFluidOut.h,
          d_A = if noEvent(portA.m_flow>0) then vleFluidA.d else vleFluidOut.d,
          d_B = if noEvent(portB.m_flow>0) then vleFluidB.d else vleFluidOut.d,
          m_flow_A = portA.m_flow,
          m_flow_B = portB.m_flow,
          q_A = if noEvent(portA.m_flow>0) then vleFluidA.q else vleFluidOut.q,
          q_B = if noEvent(portB.m_flow>0) then vleFluidB.q else vleFluidOut.q,
          effectiveFlowArea = AEffAux,
          w_A = if noEvent(portA.m_flow>0) then vleFluidA.w else vleFluidOut.w,
          w_B = if noEvent(portB.m_flow>0) then vleFluidB.w else vleFluidOut.w,
          velocity_A = if noEvent(portA.m_flow>0) then massFlowRateDensity_A/vleFluidA.d else massFlowRateDensity_A/vleFluidOut.d,
          velocity_B = if noEvent(portB.m_flow>0) then -massFlowRateDensity_A/vleFluidB.d else -massFlowRateDensity_A/vleFluidOut.d,
          dp = vleFluidA.p - vleFluidB.p,
          superheating = noEvent(max(0,noEvent(max(vleFluidB.T -vleFluidB.VLE.T_v,
          vleFluidA.T -vleFluidA.VLE.T_v)))),
          subcooling = noEvent(max(0,noEvent(max(vleFluidB.VLE.T_v- vleFluidB.T,
              vleFluidA.VLE.T_v
                              - vleFluidA.T)))),
          mass = 0,
          volume= 0) if includeDefaultSummary
          annotation (Placement(transformation(extent={{40,-80},{60,-60}}, rotation=0)));

      protected
        model GetInputs "Get enabled inputs and parameters of disabled inputs"
          extends Modelica.Blocks.Interfaces.BlockIcon;
          Modelica.Blocks.Interfaces.RealInput effectiveFlowArea_in
            "Prescribed effective flow area"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}},
                  rotation=0)));
        end GetInputs;

       SI.Area AEffAux "Effective flow area";

      protected
        GetInputs getInputs
          annotation (Placement(transformation(extent={{-20,-10},{0,10}}, rotation=0)));
        Modelica.Blocks.Sources.Constant effectiveFlowArea_in_(k=0) if not use_effectiveFlowAreaInput;

        Utilities.Units.MassFlowDensity massFlowRateDensity_A;

      equation
        portA.xi_outflow = inStream(portB.xi_outflow);
        portB.xi_outflow = inStream(portA.xi_outflow);

        portA.h_outflow = inStream(portB.h_outflow);
        portB.h_outflow = inStream(portA.h_outflow);

        portA.h_limit = -1e6; // separate pressure levels
        portB.h_limit = -1e6;

        portB.m_flow + portA.m_flow = 0 "mass balance";

        if (use_effectiveFlowAreaInput) then
          AEffAux = getInputs.effectiveFlowArea_in;
        else
          AEffAux = effectiveFlowAreaFixed;
        end if;

        if ((generateEventsAtFlowReversal and portA.p > portB.p)
          or (not generateEventsAtFlowReversal and noEvent(portA.p > portB.p))) then
          x = 2*vleFluidA.d*(portA.p - portB.p);
        else
          x = 2*vleFluidB.d*(portA.p - portB.p);
        end if;

        massFlowRateDensity_A = TIL.Utilities.Numerics.squareRootFunction(x, x0);

        portA.m_flow = massFlowRateDensity_A*AEffAux;

        connect(effectiveFlowArea_in, getInputs.effectiveFlowArea_in) annotation (Line(
              points={{0,50},{0,30},{-40,30},{-40,8},{-22,8}}, color={0,0,127}));
        connect(effectiveFlowArea_in_.y, getInputs.effectiveFlowArea_in);

        annotation (defaultComponentName="valve",Icon(coordinateSystem(
                preserveAspectRatio=true, extent={{-80,-40},{80,40}}),
                                                      graphics={
              Polygon(
                points={{-10,60},{10,60},{0,40},{-10,60}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={175,175,175},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-80,-46},{-40,-86}},
                lineColor={0,0,0},
                textString=
                     "A"),
              Text(
                extent={{40,-46},{80,-86}},
                lineColor={0,0,0},
                textString=
                     "B"),
              Bitmap(
                extent={{-80,-40},{80,40}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAKAAAABQCAIAAAARP+ljAAAABnRSTlMA/wAAAACkwsAdAAAACXBIWXMAAAsTAAALEwEAmpwYAAADSklEQVR42u3d35F0QBQFcPUlQAZkQAZkQAZkQAZkQAZkQAZkQAZkIAT7oGpqZr/pWcafdq/T1W+7tQ9+VVN7zujbiqZpSZJMioLNbCdJommaMq+ZeRgGPBfqexiGF9pfKwgCMNOlDYJAWbLAzJb2eTmO0zQNHt+Vd9M0juMoWxaY+dCmaarr+tsfWZZVFAUe6xV2URQiWl3X4zgWAk/TNE1TnuciZsMwwCyX1jAMEW2e57PgH8Bgpku7AnhedV3btv32l9GQSOgrXpdt23Vd/6+2AhjMtGi/BJ5X27a+7yM6yw21vu+3bftZ6kvgefV9D2ZZtH3fLzHaBPxgDsNQVdW3f8fzPETntaHW87y3D1NV1TAMF9LuBjyvcRzjOBYxoyHZ2FeoqhrH8TiOa112A35mFmUqx3GQqd7uqqp2pz0EGNH50FB7IWAwS6c9A3heZVkiOi8PtWVZ7vjwzwBGQ7K9ryAA/GAWRWdN07hG5znUimh93z+CVg7w3RqSvfoKYsDPzKLoHAQB6ejcNI2IVlXVE2jlA3NtSI7oKwgDL2Suqgq0hIGpR+c/Q+35tBcFJsd8Tl/BDfjBLIrOhmFIj85JkohobduWTksA+LINyfl9BWfgB7PruiLmKIrquj4h1EZRJKJ1XfdStMSAFzYkBzFL7yvuAryQueu6vWi7riNKSxj4nIbkgqH2XsDHMfOgZQL8YN4lOhdFYVmWKNSmaUqIlhXw9obk4n0FgH8zm6YpYs6y7Jk2yzIRrWmadGk5Ay9sSAj1FQD+hpk37V2Al0RnKqF2I/A/Bes+Cx/R+IjGP1kAlheTln+RjJiEogNFB6Oq8pqvVt0XGF82sAXG14VsgfGFP1tgWYea8MrOGaH280t3J5xXw0t3cvoKvDZLFfjiowHw4ruEvgJHV64OzPXw2d2BeRwfPW7cFWFgHABnC4wRDmyBMYSF7RAWjFFiO0YJg9DYDkIDLdtRhhhGynYYKWh5jhPGQPCDLjCTPxAcI/3ZjvTHpRxsL+XAtTpsr9XBxVhsL8ZCqGV7tR1o2V5OuWoiAjal62XRV7C9IBpXvFNh/jz3ad2pcPQV5BoS0N6eGaGWU3QG7Q2Y51ALWh7ML9EZfQXvhuQH5cIJCMGLx4MAAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/ValveUni.png")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}),
                  graphics),
              Documentation(info="<html>
          <p>
          See LiquidComponents -> <a href=\"Modelica:TIL.LiquidComponents.Valves.OrificeValve\">OrificeValve</a>.
          </p>
          <hr>
          </html>"));
      end OrificeValve;
      annotation(classOrder={"*","BaseClasses","Testers"});
    end Valves;
  annotation(classOrder={"*","Internals"});
  end VLEFluidComponents;

  package HeatExchangers "Component models for different heat exchanger types"
  extends TIL.Internals.ClassTypes.ComponentPackage;

    package FinAndTube "Fin and Tube HX"
      extends TIL.Internals.ClassTypes.ComponentPackage;

      package GasVLEFluid
       extends TIL.Internals.ClassTypes.ComponentPackage;

        model CrossFlowHX "Fin and tube gas vle fluid cross flow HX"
          extends
          TIL.HeatExchangers.FinAndTube.GasVLEFluid.BaseClasses.PartialHX(
               final nFinSideParallelHydraulicFlows = hxGeometry.nParallelTubes*nCells,
               final nFinSideParallelHydraulicFlowsPerCell = hxGeometry.nParallelTubes,
               final nTubeSideParallelHydraulicFlowsPerCell = hxGeometry.nTubeSideParallelHydraulicFlows,
              gasCellGeometry(length=hxGeometry.totalHXDepth),
            includePortVLEFluid = includeDefaultSummary,
            redeclare record SummaryClass = CrossSummary (
              arrays(
                final n = nCells,
                T_vle_port = TIL.Internals.getPortValuesStirredVolumeCells(vleFluidA.T, vleFluidCell.vleFluid.T, vleFluidB.T, vleFluidCell[2:end].portA.m_flow),
                T_degC_vle_port = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_vle_port),
                T_vle_cell = vleFluidCell.vleFluid.T,
                T_degC_vle_cell = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_vle_cell),
                T_wall_cell = wallCell.wallMaterial.T,
                T_degC_wall_cell = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_wall_cell),
                T_gas_portA = {if ((gasCellFlowType == "flow A-B") or ((gasCellFlowType <> "flow B-A") and noEvent(gasCell[i].portA.m_flow >= 0))) then gasCell[i].gas_inStream.T else gas_outflow[i].T for i in 1:nCells},
                T_degC_gas_portA = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_gas_portA),
                T_gas_portB = {if ((gasCellFlowType == "flow A-B") or ((gasCellFlowType <> "flow B-A") and noEvent(gasCell[i].portA.m_flow >= 0))) then gas_outflow[i].T else gasCell[i].gas_inStream.T for i in 1:nCells},
                T_degC_gas_portB = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_gas_portB),
                alpha_vle_cell = vleFluidCell.alphaAState/vleFluidCellGeometry.heatTransferArea,
                q_vle_port = TIL.Internals.getPortValuesStirredVolumeCells(vleFluidA.q, vleFluidCell.vleFluid.q, vleFluidB.q, vleFluidCell[2:end].portA.m_flow),
                p_vle_port = cat(1,{vleFluidCell[1].portA.p},vleFluidCell.portB.p),
                h_vle_port = cat(1,{noEvent(actualStream(vleFluidCell[1].portA.h_outflow))},noEvent(actualStream(vleFluidCell.portB.h_outflow))),
                d_vle_port = TIL.Internals.getPortValuesStirredVolumeCells(vleFluidA.d, vleFluidCell.vleFluid.d, vleFluidB.d, vleFluidCell[2:end].portA.m_flow),
                m_flow_vle_port = cat(1,{vleFluidCell[1].portA.m_flow},-vleFluidCell.portB.m_flow),
                w_vle_port = summary.arrays.m_flow_vle_port ./ summary.arrays.d_vle_port ./ vleFluidCellGeometry.hydraulicCrossSectionalArea ./ hxGeometry.nTubeSideParallelHydraulicFlows,
                p_gas_portA = gasCell.portA.p,
                p_gas_portB = gasCell.portB.p,
                h_gas_portA = noEvent(actualStream(gasCell.portA.h_outflow)),
                h_gas_portB = noEvent(actualStream(gasCell.portB.h_outflow)),
                alpha_gas_cell = gasCell.heatTransfer.alphaA/gasCellGeometry.heatTransferArea)));

          /****************** Connectors *******************/

          TIL.Connectors.VLEFluidPort portB_vle(final vleFluidType=vleFluidType)
            "VLEFluid portB"
            annotation (Placement(transformation(extent={{130,-10},{150,10}}, rotation=
                    0)));
          TIL.Connectors.VLEFluidPort portA_vle(final vleFluidType=vleFluidType)
            "VLEFluid portA"
            annotation (Placement(transformation(extent={{-150,-10},{-130,10}},
                  rotation=0)));
          TIL.Connectors.GasPort portA_gas(final gasType=gasType) "Gas portA"
            annotation (Placement(transformation(extent={{-10,130},{10,150}}, rotation=
                    0)));
          TIL.Connectors.GasPort portB_gas(final gasType=gasType) "Gas portB"
            annotation (Placement(transformation(extent={{-10,-150},{10,-130}},
                  rotation=0)));

          /****************** Splitter and joiner *******************/

        protected
          SplitterJoiner.GasSplitterJoiner gasSplitterJoiner(
            final nPorts1 = nCells,
            final nPorts2 = 1,
            final gasType=gasType,
            final cellFlowType=gasCellFlowType)
            annotation (Placement(transformation(extent={{-26,-50},{26,-30}})));

        /********* Summary ***************/

        protected
          parameter Boolean includeSummaryArrays = true
            "Obsolete & unused parameter for array entries in summary"
         annotation(Dialog(tab="Advanced", group="Summary"));

          record CrossSummary
            extends Summary;

            replaceable Arrays arrays;

          protected
            record Arrays
              parameter Integer n;

              input SI.Temperature[n + 1] T_vle_port "Temperature at port";
              input SI.Temp_C[n + 1] T_degC_vle_port "Temperature at port";
              input SI.Temperature[n] T_vle_cell "Temperature in cell";
              input SI.Temp_C[n] T_degC_vle_cell "Temperature in cell";
              input SI.MassFraction[n + 1] q_vle_port
                "Vapor quality (steam mass fraction) at port";
              input SI.Pressure[n + 1] p_vle_port "Pressure at port";
              input SI.SpecificEnthalpy[n + 1] h_vle_port "Specific enthalpy at port";
              input SI.Density[n + 1] d_vle_port "Density at port";
              input SI.MassFlowRate[n + 1] m_flow_vle_port "Mass flow rate at port";
              input SI.Velocity[n + 1] w_vle_port "Flow velocity at port";
              input SI.CoefficientOfHeatTransfer[n] alpha_vle_cell
                "Heat transfer coefficient";

              input SI.Temperature[n] T_wall_cell "Temperature in cell";
              input SI.Temp_C[n] T_degC_wall_cell "Temperature in cell";

              input SI.Temperature[n] T_gas_portA "Temperature at port A";
              input SI.Temp_C[n] T_degC_gas_portA "Temperature at port A";
              input SI.Temperature[n] T_gas_portB "Temperature at port B";
              input SI.Temp_C[n] T_degC_gas_portB "Temperature at port B";
              input SI.Pressure[n] p_gas_portA "Pressure at port A";
              input SI.Pressure[n] p_gas_portB "Pressure at port B";
              input SI.SpecificEnthalpy[n] h_gas_portA "Specific enthalpy at port A";
              input SI.SpecificEnthalpy[n] h_gas_portB "Specific enthalpy at port B";
              input SI.CoefficientOfHeatTransfer[n] alpha_gas_cell
                "Heat transfer coefficient";

            end Arrays;
          end CrossSummary;

        protected
          TILMedia.Gas_ph[nCells] gas_outflow(
            final p = {if ((gasCellFlowType == "flow A-B") or ((gasCellFlowType <> "flow B-A") and noEvent(gasCell[i].portA.m_flow >= 0))) then gasCell[i].portB.p else gasCell[i].portA.p for i in 1:nCells},
            final h = {if ((gasCellFlowType == "flow A-B") or ((gasCellFlowType <> "flow B-A") and noEvent(gasCell[i].portA.m_flow >= 0))) then gasCell[i].portB.h_outflow else gasCell[i].portA.h_outflow for i in 1:nCells},
            final xi = {if ((gasCellFlowType == "flow A-B") or ((gasCellFlowType <> "flow B-A") and noEvent(gasCell[i].portA.m_flow >= 0))) then gasCell[i].portB.xi_outflow else gasCell[i].portA.xi_outflow for i in 1:nCells},
            each final gasType=gasType,
            each computeTransportProperties=false) if
                                           includeDefaultSummary
            annotation (Placement(transformation(extent={{-50,-100},{-30,-80}},rotation=
                   0)));

        equation
        // Connect wall and vleFluid cells with eachother
          for i in 1:nCells-1 loop
            connect(vleFluidCell[i].portB, vleFluidCell[i+1].portA);
            connect(wallCell[i].portE, wallCell[i+1].portW);
          end for;

          connect(vleFluidCell[1].portA, portA_vle) annotation (Line(
              points={{-10,40},{-100,40},{-100,0},{-140,0}},
              color={153,204,0},
              thickness=0.5));
          connect(vleFluidCell[nCells].portB, portB_vle) annotation (Line(
              points={{10,40},{100,40},{100,0},{140,0}},
              color={153,204,0},
              pattern=LinePattern.Solid,
              thickness=0.5));
          connect(wallCell.portS, gasCell.heatPort) annotation (Line(
              points={{0,-10},{0,-30.2}},
              color={204,0,0},
              pattern=LinePattern.Solid,
              thickness=0.5));
          connect(vleFluidCell.heatPort, wallCell.portN) annotation (Line(
              points={{0,30},{0,10}},
              color={204,0,0},
              pattern=LinePattern.Solid,
              thickness=0.5));

          connect(gasSplitterJoiner.outlet, portB_gas) annotation (Line(
              points={{26,-40},{40,-40},{40,-100},{0,-100},{0,-140}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          connect(gasSplitterJoiner.inlet, portA_gas) annotation (Line(
              points={{-26,-40},{-40,-40},{-40,100},{0,100},{0,140}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          connect(gasSplitterJoiner.outlets[:, 1], gasCell.portA) annotation (Line(
              points={{-14,-40},{-10,-40}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          connect(gasSplitterJoiner.inlets[:, 1], gasCell.portB) annotation (Line(
              points={{14,-40},{10,-40}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          annotation(Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td colspan=2>VLE fluid</td>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient (default) or steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>h, xi, alphaAState, pressureDropState (default) or derivative of pressure in the PressureStateElement</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure drop or isobaric</td>
        </tr>
        <tr>
        <td colspan=2>Wall</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>T</td>
        </tr>
        <tr>
        <td colspan=2>Gas</td>
        </tr>
        <tr>
        <td>mass balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>differential states:</td><td>-</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure drop or isobaric</td>
        </tr>
        </table>
<p>
This is the base model for a cross flow heat exchanger. The following figure illustrates
the flow situation.<br><br>
</p>
<img src=\"modelica://TIL/Images/TIL3_InfoView_HXStructure_CrossFlow.png\" width=\"600\">
</html>"),  Diagram(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-140,-140},{140,140}},
                initialScale=0.1), graphics),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-140,-140},{140,140}},
                initialScale=0.1), graphics={
                Bitmap(extent={{-140,-140},{140,140}},
                  imageSource=
                      "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HS7eGJsTG9WNyUJI7Rhkq2tL6WePPPNgmz6SO+5q3OHSsnhsVHFZPL1ubNFkYbEhF33XeQ/sXh77o39yWGxIDfrUq3Qz53aZu8pig5CGST+QKP3k/npXruoPqe+RJ93Mrv7fmYTUkCHvIZv6a0BnFdKWfq+ybW6y6ERIrpE2EQPo2X0r6fteek8W1VtRg2JD+r6959/NXFGRxYZaBtmOrY+9+Mhax1auYnNio4orlh2bLISU1yCbykf/5Kn4gUSdnCxqPM1zjVTdjb+uXbh6stH/9UNT10gl568s79uxZJnbYkNXzCgkLDaAayQQEggJEBIICYQECAmEBEICIQFCAiGBkEBIgJBASCAkEBIgJBASCAmEBAgJhARCAiEBQgIhgZBASICQQEggJEBIICQQEggJEBIICYQEQgKEBEICIYGQACGBkEBIICRASCAkEBIICRASCAmEBEIChARCAiEBQgIhgZBASICQQEggJBASICQQEggJhAQICYQEQgIhAUICIYGQQEiAkEBIICRASCAkEBIICRASCAmEBEIChARCAiGBkAAhgZBASCAkQEggJBASCAkQEggJhARCAoQEQgIhAUICIYGQQEgAAAAAA/wjwAAuFnEcIolTZQAAAABJRU5ErkJggg==",
                  fileName="modelica://TIL/Images/CrossFlowHX_VLEGas.png"),
                Text(
                  extent={{-46,110},{194,70}},
                  lineColor={153,204,0},
                  textString=
                       "(%pressureStateID)"),
                Text(
                  extent={{-140,0},{-100,-40}},
                  lineColor={0,0,0},
                  textString=
                       "1"),
                Text(
                  extent={{100,0},{140,-40}},
                  lineColor={0,0,0},
                  textString=
                       "n"),
                Text(
                  extent={{-132,130},{-92,90}},
                  lineColor={0,0,0},
                  textString=
                       "A"),
                Text(
                  extent={{92,-92},{132,-132}},
                  lineColor={0,0,0},
                  textString=
                       "B")}));
        end CrossFlowHX;

        package BaseClasses
        extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialHX "Partial base class for fin-and-tube heat exchangers"

          /*********************** SIM *************************/

            parameter TILMedia.GasTypes.BaseGas           gasType = sim.gasType1
              "Gas type" annotation (Dialog(tab="SIM",group="SIM"),choices(
              choice=sim.gasType1 "Gas 1 as defined in SIM",
              choice=sim.gasType2 "Gas 2 as defined in SIM",
              choice=sim.gasType3 "Gas 3 as defined in SIM"));
            parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
              "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
              choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
              choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
              choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));
          protected
            outer SystemInformationManager sim "System information manager";
            TIL.Internals.SimPort simPort;

          /*********** Connectors *************************/
          public
            TIL.Connectors.VLEFluidPort portB_vle(final vleFluidType=vleFluidType);
            TIL.Connectors.VLEFluidPort portA_vle(final vleFluidType=vleFluidType);
            TIL.Connectors.GasPort portA_gas(final gasType=gasType);
            TIL.Connectors.GasPort portB_gas(final gasType=gasType);

           /****************** Geometry, Discretization *******************/
            inner replaceable parameter TIL.HeatExchangers.FinAndTube.Geometry.KKI001
              hxGeometry
              constrainedby
              TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              "Geometry of heat exchanger"
              annotation(Dialog(group="General"),choicesAllMatching=true,
              Placement(transformation(extent={{-100,100},{-80,120}}, rotation=0)));

            parameter Integer nCells(min=1)=1 "Discretization number of cells";

            parameter Integer nFinSideParallelHydraulicFlows(min=1)
              "Total number of entering hydraulic flows"
              annotation (Dialog(tab="Discretization",group="Fin Side"));
            parameter Integer nFinSideParallelHydraulicFlowsPerCell(min=1)
              "Number of hydraulic flows for a single cell"
              annotation (Dialog(tab="Discretization",group="Fin Side"));
            parameter Integer nTubeSideParallelHydraulicFlowsPerCell(min=1)
              "Number of hydraulic flows for a single cell"
              annotation (Dialog(tab="Discretization",group="Tube Side"));

          protected
            TIL.Cells.Geometry.FluidCellGeometry vleFluidCellGeometry(
              length=hxGeometry.tubeSidePathLength/nCells,
              volume=hxGeometry.totalTubeInnerVolume/nCells,
              heatTransferArea=hxGeometry.totalTubeSideHeatTransferArea/nCells,
              hydraulicCrossSectionalArea=PI*hxGeometry.tubeInnerDiameter*hxGeometry.tubeInnerDiameter/4.0,
              nParallelHydraulicFlows= nTubeSideParallelHydraulicFlowsPerCell);

            TIL.Cells.Geometry.WallCellGeometry wallCellGeometry(
              length=hxGeometry.tubeSidePathLength/nCells,
              volume=(hxGeometry.totalTubeOuterVolume-hxGeometry.totalTubeInnerVolume)/nCells);

            TIL.Cells.Geometry.FluidCellGeometry gasCellGeometry(
              volume=hxGeometry.totalFinnedVolume*hxGeometry.voidRatio/nCells,
              heatTransferArea=hxGeometry.totalFinSideHeatTransferArea/nCells,
              hydraulicCrossSectionalArea=hxGeometry.totalFinSideCrossSectionalArea*hxGeometry.areaRatio/nFinSideParallelHydraulicFlows,
              finHeatTransferAreaRatio=hxGeometry.finSideHeatTransferAreaRatio,
              nParallelHydraulicFlows=nFinSideParallelHydraulicFlowsPerCell);

          /****************** VLEFluid *******************/
          public
            parameter TIL.Internals.PressureStateID pressureStateID=1 "Pressure state ID"
              annotation(Dialog(tab="SIM",group="SIM"));

            SI.Mass cumulatedVLEFluidMass=sum(vleFluidCell.mass)
              "Cumulated vleFluid mass";
            SI.Volume cumulatedVLEFluidVolume=sum(vleFluidCell.cellGeometry.volume)
              "Cumulated vleFluid volume";

          protected
            Cells.VLEFluidCell[nCells] vleFluidCell(
              each cellGeometry = vleFluidCellGeometry,
              each final dpdt=sim.dpdtPort[pressureStateID].dpdt,
              each final vleFluidType=vleFluidType,
              redeclare each final model HeatTransferModel =
                  TubeSideHeatTransferModel,
              redeclare each final model PressureDropModel =
                  TubeSidePressureDropModel,
              each final m_flowStart=m_flowVLEFluidStart,
              each final pStart=pVLEFluidStart,
              each final orientation=cellOrientation,
              each final pressureDropInitial = pressureDropInitialVLEFluid/nCells,
              each final fixedPressureDropInitial = fixedPressureDropInitialVLEFluid,
              each final useFalseDynamics = useFalseDynamics,
              each final falseDynamicsTimeConstant = falseDynamicsTimeConstant,
              each final removeSingularity=sim.removeSingularity,
              each final generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal,
              each final alphaAInitial = if (initHeatTransfer=="alphaA") then alphaAInitialVLEFluid/nCells
                                         else alphaInitialVLEFluid*hxGeometry.totalTubeSideHeatTransferArea/nCells,
              each final alphaAStateTimeConstant = alphaAStateTimeConstant,
              each useAlphaAState=if (TIL.Internals.AlphaAStateChoice.useAlphaAState ==
                  alphaAStateChoice) then true elseif (TIL.Internals.AlphaAStateChoice.doNotUseAlphaAState
                   == alphaAStateChoice) then false else vleFluidCell[1].heatTransfer.useAlphaAState)
              "VLEFluid cells"
              annotation (Placement(transformation(extent={{-10,30},{10,50}}, rotation=0)));

          public
            replaceable model TubeSideHeatTransferModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ConstantAlpha
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.PartialVLEFluidHeatTransfer
              "Heat transfer model" annotation(Dialog(group="VLEFluid"),choicesAllMatching=true);

            replaceable model TubeSidePressureDropModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.PartialVLEFluidPressureDrop
              "VLEFluid pressure drop model" annotation(Dialog(group="VLEFluid"),choicesAllMatching=true);

            parameter TIL.Cells.Internals.VLEFluidCellOrientationType cellOrientation= "A"
              "Selection of cell orientation for equations of mDotHydraulic and pressure"
                 annotation(Dialog(tab="Advanced",group="VLEFluid Cells"));

            parameter Boolean useFalseDynamics=true
              "Sets pressure drop as differential state; True, if false dynamics should be used"
              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

            parameter Real falseDynamicsTimeConstant=0.1
              "Time constant of false dynamics for pressureDropState" annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

            parameter TIL.Internals.AlphaAStateChoice
                      alphaAStateChoice = TIL.Internals.AlphaAStateChoice.definedByHeatTransferCorrelation
              "choice 1: force use; choice 2: force use not; choice 3: defined by heat transfer"
              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

            parameter Real alphaAStateTimeConstant=1 "Time constant for alphaAState"
                                              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

          /****************** Wall *******************/

          protected
            Cells.WallCell[nCells] wallCell(
              each cellGeometry = wallCellGeometry,
              each TInitialWall=TInitialWall,
              redeclare each final model WallMaterial=WallMaterial,
              redeclare each final model HeatTransferModel =
                  WallHeatConductionModel,
              each fixedTInitialWall=fixedTInitialWall) "Wall cells"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));

          public
            replaceable model WallMaterial = TILMedia.SolidTypes.TILMedia_Steel
              constrainedby TILMedia.SolidTypes.BaseSolid "Wall material"
              annotation(Dialog(group="Tube"),choicesAllMatching=true);
            replaceable model WallHeatConductionModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.GeometryBasedConduction
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.PartialHeatTransfer
              "Heat transfer model" annotation(Dialog(group="Tube"),choicesAllMatching=true);

          /****************** Fin *******************/

            replaceable model FinMaterial =
                TILMedia.SolidTypes.TILMedia_Aluminum
              constrainedby TILMedia.SolidTypes.BaseSolid "Solid fluid type"
            annotation(Dialog(group="Fin"),choicesAllMatching=true);

            replaceable model FinEfficiencyModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.Schmidt
                constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.PartialFinEfficiency
              "Fin efficiency model"  annotation(Dialog(group="Fin"),choicesAllMatching=true);

          /****************** Gas *******************/

          protected
            TIL.Cells.GasCell[nCells] gasCell(
              each cellGeometry = gasCellGeometry,
              each final gasType=gasType,
              each final cellFlowType=gasCellFlowType,
              each final useFalseDynamics = useFalseDynamicsGas,
              each final falseDynamicsTimeConstant = falseDynamicsTimeConstantGas,
              each final pressureDropInitial = pressureDropInitialGas,
              each final fixedPressureDropInitial = fixedPressureDropInitialGas,
              redeclare each final model HeatTransferModel =
                  FinSideHeatTransferModel,
              redeclare each final model PressureDropModel =
                  FinSidePressureDropModel,
              redeclare final model FinEfficiencyModel =
                                          FinEfficiencyModel,
              redeclare each final model FinMaterial = FinMaterial) "Gas cells"
              annotation (Placement(transformation(extent={{-10,-50},{10,-30}}, rotation=
                      0)));

          public
            replaceable model FinSideHeatTransferModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.ConstantAlpha
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.PartialHeatTransfer
              "Heat transfer model" annotation(Dialog(group="Gas"),choicesAllMatching=true);

            replaceable model FinSidePressureDropModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSidePressureDrop.ZeroPressureDrop
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSidePressureDrop.PartialPressureDrop
              "Pressure drop model" annotation(Dialog(group="Gas"),choicesAllMatching=true);

            parameter TIL.Internals.CellFlowType gasCellFlowType = "flow A-B"
              "Allowed flow directions in gas cell" annotation(Dialog(tab="Advanced",group="Gas Cells"));

            /****************** Start values *******************/

            parameter SI.MassFlowRate m_flowVLEFluidStart=0.05
              "Start value for mass flow rate"
              annotation(Dialog(tab="Start Values",group="VLEFluid"));
            parameter SI.AbsolutePressure pVLEFluidStart=1e5 "Start value for pressure"
              annotation(Dialog(tab="Start Values",group="VLEFluid"));

            /****************** Initialization *******************/

            inner parameter TIL.Internals.InitializationMethods initVLEFluid=
                                                                    "constantEnthalpy"
              "Initialization method" annotation(choicesAllMatching,
              Dialog(tab="Initialization", group="VLEFluid"));

          // Values for constant enthalpy
            inner parameter SI.SpecificEnthalpy hInitialVLEFluid=300e3
              "Initial value for specific enthalpy"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="constantEnthalpy")));

          // Values for linear enthalpy profile
            parameter SI.SpecificEnthalpy hInitialVLEFluid_Cell1=250e3
              "Initial value for specific enthalpy of cell 1"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="linearEnthalpyDistribution")));
            parameter SI.SpecificEnthalpy hInitialVLEFluid_CellN=350e3
              "Initial value for specific enthalpy of cell n"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="linearEnthalpyDistribution")));

          // Values for user specific specific enthalpies
            parameter SI.SpecificEnthalpy[nCells] hInitialUserValues=ones(nCells)*hInitialVLEFluid
              "Initial values for specific enthalpies"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="userSpecifiedValues")));

          // Values for constant temperature
            parameter SI.Temperature TInitialVLEFluid=298.15
              "Initial value for temperature"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="constantTemperature")));

            parameter SI.Pressure pressureDropInitialVLEFluid = 0
              "Initial value for pressure drop"
              annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization"));

            parameter Boolean fixedPressureDropInitialVLEFluid = true
              "If true, force usage of initial value" annotation(Dialog(tab="Advanced Initialization", group="VLEFluid"));

            parameter SI.Temperature TInitialWall = 298.15
              "Initial value for temperature"
              annotation(Dialog(tab="Initialization", group="Wall"));

            parameter String initHeatTransfer="alphaA"
              "Choice of initialization variable"
              annotation(choices(
                  choice= "alpha" "use alphaInitial for initialization",
                  choice= "alphaA" "use alphaAInitial for initialization"),
                  Dialog(tab="Advanced Initialization", group="VLEFluid"));

            parameter SI.CoefficientOfHeatTransfer alphaInitialVLEFluid = 0
              "Initial value for alpha"
              annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization",enable=(initHeatTransfer=="alpha")));

            parameter Real alphaAInitialVLEFluid = 0 "Initial value for alphaA"
              annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization",enable=(initHeatTransfer=="alphaA")));

            parameter Real[vleFluidType.nc] mixingRatioInitialVLEFluid = vleFluidType.defaultMixingRatio
              "Initial value for mixing ratio" annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization"));

            parameter SI.Pressure pressureDropInitialGas = 0
              "Initial value for pressure drop"
              annotation (Dialog(group="Gas", tab="Advanced Initialization"));
            parameter Boolean fixedPressureDropInitialGas = true
              "If true, force usage of initial value"
              annotation (Dialog(group="Gas", tab="Advanced Initialization"));

            parameter Boolean useFalseDynamicsGas = true
              "Sets pressure drop as differential state; True, if false dynamics should be used"
              annotation (Dialog(group="Gas Cells", tab="Advanced"));
            parameter Real falseDynamicsTimeConstantGas=0.1
              "Time constant of false dynamics for pressureDropState"
              annotation (Dialog(group="Gas Cells", tab="Advanced"));

          protected
            TILMedia.Gas_ph  gasA(
              final gasType=gasType,
              final h = noEvent(actualStream(portA_gas.h_outflow)),
              final xi = noEvent(actualStream(portA_gas.xi_outflow)),
              final p = portA_gas.p,
              computeTransportProperties=false) if
                                        includePortVLEFluid "Inlet gas"
              annotation (Placement(transformation(extent={{-80,-80},{-60,-60}}, rotation=
                     0)));

            TILMedia.Gas_ph  gasB(
              final gasType=gasType,
              final h = noEvent(actualStream(portB_gas.h_outflow)),
              final xi = noEvent(actualStream(portB_gas.xi_outflow)),
              final p = portB_gas.p,
              computeTransportProperties=false) if
                                        includePortVLEFluid "Outlet gas"
              annotation (Placement(transformation(extent={{60,-80},{80,-60}}, rotation=0)));

            TILMedia.VLEFluid_ph vleFluidA(
              final vleFluidType =  vleFluidType,
              final h = noEvent(actualStream(portA_vle.h_outflow)),
              final xi = noEvent(actualStream(portA_vle.xi_outflow)),
              final p = portA_vle.p) if includePortVLEFluid "Inlet vleFluid"
              annotation (Placement(transformation(extent={{-80,60},{-60,80}}, rotation=0)));

            TILMedia.VLEFluid_ph vleFluidB(
              final vleFluidType =  vleFluidType,
              final h = noEvent(actualStream(portB_vle.h_outflow)),
              final xi = noEvent(actualStream(portB_vle.xi_outflow)),
              final p = portB_vle.p) if includePortVLEFluid "Outlet vleFluid"
              annotation (Placement(transformation(extent={{60,60},{80,80}}, rotation=0)));

            /*******************Summary**********************/
          public
            parameter Boolean includeDefaultSummary = true
              "Include summary record in model results"
           annotation(Dialog(tab="Advanced", group="Summary"));

          protected
            parameter Boolean includePortVLEFluid = includeDefaultSummary;

          protected
            record Summary
              extends TIL.Internals.ClassTypes.Record;

              SI.Pressure p_vle_A "Pressure at port A";
              SI.Pressure p_vle_B "Pressure at port B";
              SI.Temperature T_vle_A "Temperature at port A";
              SI.Temperature T_vle_B "Temperature at port B";
              SI.Temp_C T_degC_vle_A "Temperature at port A";
              SI.Temp_C T_degC_vle_B "Temperature at port B";
              SI.MassFraction q_vle_A "Vapor quality (steam mass fraction) at port A";
              SI.MassFraction q_vle_B "Vapor quality (steam mass fraction) at port B";
              SI.SpecificEnthalpy h_vle_A "Specific enthalpy at port A";
              SI.SpecificEnthalpy h_vle_B "Specific enthalpy at port B";
              SI.Density d_vle_A "Density at port A";
              SI.Density d_vle_B "Density at port B";
              SI.MassFlowRate m_flow_vle_A "Mass flow rate at port A";
              SI.MassFlowRate m_flow_vle_B "Mass flow rate at port B";
              SI.Pressure dp_vle "Total pressure drop";
              SI.HeatFlowRate Q_flow_vle "Total heat flow rate";
              SI.TemperatureDifference superheating "Superheating";
              SI.TemperatureDifference subcooling "Subcooling";
              SI.Mass mass_vle "Total fluid mass";
              SI.Volume volume_vle "Total fluid volume";
              SI.CoefficientOfHeatTransfer alpha_average_vle
                "Average coefficient of heat transfer";

              SI.Pressure p_gas_A "Pressure at port A";
              SI.Pressure p_gas_B "Pressure at port B";
              SI.Temperature T_gas_A "Temperature at port A";
              SI.Temperature T_gas_B "Temperature at port B";
              SI.Temp_C T_degC_gas_A "Temperature at port A";
              SI.Temp_C T_degC_gas_B "Temperature at port B";
              SI.SpecificEnthalpy h_gas_A "Specific enthalpy at port A";
              SI.SpecificEnthalpy h_gas_B "Specific enthalpy at port B";
              SI.MassFlowRate m_flow_gas_A "Mass flow rate at port A";
              SI.MassFlowRate m_flow_gas_B "Mass flow rate at port B";
              SI.Velocity w_gas_A "Flow velocity at port A";
              SI.Velocity w_gas_B "Flow velocity at port B";
              SI.Pressure dp_gas "Total pressure drop";
              SI.HeatFlowRate Q_flow_gas "Total heat flow rate";
              SI.CoefficientOfHeatTransfer alpha_average_gas
                "Average coefficient of heat transfer";

              SI.Mass mass_component "Solid mass of component";

              parameter Geometry.Summary geometry;

            end Summary;

            replaceable record SummaryClass = Summary;

          public
            SummaryClass summary(
              p_vle_A = vleFluidA.p,
              p_vle_B = vleFluidB.p,
              T_vle_A = vleFluidA.T,
              T_vle_B = vleFluidB.T,
              T_degC_vle_A = Modelica.SIunits.Conversions.to_degC(vleFluidA.T),
              T_degC_vle_B = Modelica.SIunits.Conversions.to_degC(vleFluidB.T),
              q_vle_A=vleFluidA.q,
              q_vle_B=vleFluidB.q,
              h_vle_A = vleFluidA.h,
              h_vle_B = vleFluidB.h,
              d_vle_A = vleFluidA.d,
              d_vle_B = vleFluidB.d,
              m_flow_vle_A = portA_vle.m_flow,
              m_flow_vle_B = portB_vle.m_flow,
              dp_vle = vleFluidA.p - vleFluidB.p,
              Q_flow_vle = sum(vleFluidCell.heatPort.Q_flow),
              superheating = noEvent(max(0, noEvent(max(vleFluidB.T -vleFluidB.VLE.T_v, vleFluidA.T -
                  vleFluidA.VLE.T_v)))),
              subcooling = noEvent(max(0, noEvent(max(vleFluidB.VLE.T_l- vleFluidB.T,
                  vleFluidA.VLE.T_l                                                                          - vleFluidA.T)))),
              mass_vle = cumulatedVLEFluidMass,
              volume_vle = cumulatedVLEFluidVolume,
              alpha_average_vle = sum(vleFluidCell.alphaAState)/hxGeometry.totalTubeSideHeatTransferArea,
              p_gas_A = gasA.p,
              p_gas_B = gasB.p,
              T_gas_A = gasA.T,
              T_gas_B = gasB.T,
              T_degC_gas_A = Modelica.SIunits.Conversions.to_degC(gasA.T),
              T_degC_gas_B = Modelica.SIunits.Conversions.to_degC(gasB.T),
              h_gas_A = gasA.h,
              h_gas_B = gasB.h,
              m_flow_gas_A = portA_gas.m_flow,
              m_flow_gas_B = portB_gas.m_flow,
              w_gas_A = portA_gas.m_flow/gasA.d/hxGeometry.totalFinSideCrossSectionalArea,
              w_gas_B = -portB_gas.m_flow/gasB.d/hxGeometry.totalFinSideCrossSectionalArea,
              dp_gas = gasA.p - gasB.p,
              Q_flow_gas = sum(gasCell.heatPort.Q_flow),
              alpha_average_gas = sum(gasCell.heatTransfer.alphaA)/hxGeometry.totalFinSideHeatTransferArea,
              mass_component = sum(wallCell.geometricMass),
              geometry = hxGeometry.summary) if
              includeDefaultSummary
              annotation (Placement(transformation(extent={{80,100},{100,120}}, rotation=
                      0)));

          protected
            parameter String hxType="FinAndTube" "HX type";

          public
            parameter Boolean fixedTInitialWall=true
              "if true, force usage of initial value TInitialWall"
              annotation (Dialog(tab="Advanced Initialization", group="Wall"));
          initial equation
            if (initVLEFluid == "constantEnthalpy") then
              vleFluidCell.vleFluid.h = hInitialVLEFluid*ones(nCells);
            elseif (initVLEFluid == "linearEnthalpyDistribution") then
              if nCells > 1 then
                vleFluidCell.vleFluid.h = linspace(hInitialVLEFluid_Cell1,hInitialVLEFluid_CellN,nCells);
              else
                terminate("Initialization with linear enthalpy distrubution requires discretization >1!");
              end if;
            elseif (initVLEFluid == "constantTemperature") then
              vleFluidCell.vleFluid.h = TILMedia.VLEFluidFunctions.specificEnthalpy_pTxi(vleFluidType, vleFluidCell.vleFluid.p,TInitialVLEFluid,mixingRatioInitialVLEFluid[1:end-1]/sum(mixingRatioInitialVLEFluid));
            elseif (initVLEFluid == "steadyState") then
              der(vleFluidCell.vleFluid.h) = zeros(nCells);
            elseif (initVLEFluid == "userSpecifiedValues") then
              vleFluidCell.vleFluid.h = hInitialUserValues;
            else
              terminate("Initialization method not yet implemented in Fin-and-tube heat exchanger!");
            end if;

            for i in 1 : nCells loop
              vleFluidCell[i].vleFluid.xi = mixingRatioInitialVLEFluid[1:end-1]/sum(mixingRatioInitialVLEFluid);
            end for;

            assert(PI*hxGeometry.tubeOuterDiameter*hxGeometry.tubeOuterDiameter/4 <
                  hxGeometry.serialTubeDistance*hxGeometry.parallelTubeDistance, "Distance from tube to tube is smaller than outer diameter of the tube;
         Reduce tubeInnerDiameter or enlarge serialTubeDistance and parallelTubeDistance to solve the problem");
            assert(hxGeometry.geometryIsValid, "Geometry is not valid");
            assert(sim.dpdtPort[pressureStateID].counter>0.5,"Pressure state ID is not valid");

          equation
            connect(sim.fluidPort[vleFluidType.ID],simPort.vleFluidPort);
            simPort.vleFluidMass
                             =cumulatedVLEFluidMass;
            simPort.vleFluidVolume
                               =cumulatedVLEFluidVolume;

              annotation(Dialog(group="General"),
              Icon(coordinateSystem(
                  preserveAspectRatio=false,
                  extent={{-140,-140},{140,140}},
                  initialScale=0.1), graphics),
              Diagram(coordinateSystem(
                  preserveAspectRatio=false,
                  extent={{-140,-140},{140,140}},
                  initialScale=0.1), graphics));
          end PartialHX;
        end BaseClasses;
        annotation(classOrder={"ParallelFlowHX","CrossFlowHX","PortArrayCrossFlowHX","*","BaseClasses","Testers"});
      end GasVLEFluid;

      package MoistAirVLEFluid
      extends TIL.Internals.ClassTypes.ComponentPackage;

        model CrossFlowHX "Fin and tube moist air vle fluid cross flow HX"
         extends
          TIL.HeatExchangers.FinAndTube.MoistAirVLEFluid.BaseClasses.PartialHX(
               final nFinSideParallelHydraulicFlows = hxGeometry.nParallelTubes*nCells,
               final nFinSideParallelHydraulicFlowsPerCell = hxGeometry.nParallelTubes,
               final nTubeSideParallelHydraulicFlowsPerCell = hxGeometry.nTubeSideParallelHydraulicFlows,
              moistAirCellGeometry(length=hxGeometry.totalHXDepth),
            includePortVLEFluid = includeDefaultSummary,
            redeclare record SummaryClass = CrossSummary (
              arrays(
                final n = nCells,
                final m = gasType.nc-1,
                T_vle_port = TIL.Internals.getPortValuesStirredVolumeCells(vleFluidA.T, vleFluidCell.vleFluid.T, vleFluidB.T, vleFluidCell[2:end].portA.m_flow),
                T_degC_vle_port = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_vle_port),
                T_vle_cell = vleFluidCell.vleFluid.T,
                T_degC_vle_cell = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_vle_cell),
                T_finSideWallSurface_cell = moistAirCell.wallSurfaceTemperature,
                T_degC_finSideWallSurface_cell = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_finSideWallSurface_cell),
                T_wall_cell = moistAirCell.wallMaterial.T,
                T_degC_wall_cell = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_wall_cell),
                T_air_portA = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAirCell[i].moistAir_inStream.T else moistAir_outflow[i].T for i in 1:nCells},
                T_degC_air_portA = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_air_portA),
                T_air_portB = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAir_outflow[i].T else moistAirCell[i].moistAir_inStream.T for i in 1:nCells},
                T_degC_air_portB = Modelica.SIunits.Conversions.to_degC(summary.arrays.T_air_portB),
                alpha_vle_cell = vleFluidCell.alphaAState/vleFluidCellGeometry.heatTransferArea,
                q_vle_port = TIL.Internals.getPortValuesStirredVolumeCells(vleFluidA.q, vleFluidCell.vleFluid.q, vleFluidB.q, vleFluidCell[2:end].portA.m_flow),
                p_vle_port = cat(1,{vleFluidCell[1].portA.p},vleFluidCell.portB.p),
                h_vle_port = cat(1,{noEvent(actualStream(vleFluidCell[1].portA.h_outflow))},noEvent(actualStream(vleFluidCell.portB.h_outflow))),
                d_vle_port = TIL.Internals.getPortValuesStirredVolumeCells(vleFluidA.d, vleFluidCell.vleFluid.d, vleFluidB.d, vleFluidCell[2:end].portA.m_flow),
                m_flow_vle_port = cat(1,{vleFluidCell[1].portA.m_flow},-vleFluidCell.portB.m_flow),
                w_vle_port = summary.arrays.m_flow_vle_port ./ summary.arrays.d_vle_port ./ vleFluidCellGeometry.hydraulicCrossSectionalArea ./ hxGeometry.nTubeSideParallelHydraulicFlows,
                p_air_portA = moistAirCell.moistAirPortA.p,
                p_air_portB = moistAirCell.moistAirPortB.p,
                h_air_portA = noEvent(actualStream(moistAirCell.moistAirPortA.h_outflow)),
                h_air_portB = noEvent(actualStream(moistAirCell.moistAirPortB.h_outflow)),
                xi_air_portA = {noEvent(actualStream(moistAirCell[:].moistAirPortA.xi_outflow[i])) for i in 1:gasType.nc-1},
                xi_air_portB = {noEvent(actualStream(moistAirCell[:].moistAirPortB.xi_outflow[i])) for i in 1:gasType.nc-1},
                xis_air_wall_cell = moistAirCell.moistAirWall.xi_s,
                phi_air_portA = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAirCell[i].moistAir_inStream.phi else moistAir_outflow[i].phi for i in 1:nCells},
                phi_air_portB = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAir_outflow[i].phi else moistAirCell[i].moistAir_inStream.phi for i in 1:nCells},
                alpha_air_cell = moistAirCell.heatTransfer.alphaA/moistAirCellGeometry.heatTransferArea)));

          /****************** Connectors *******************/

          TIL.Connectors.VLEFluidPort portB_vle( final vleFluidType=vleFluidType)
            "VLEFluid portB"
            annotation (Placement(transformation(extent={{130,-10},{150,10}}, rotation=
                    0)));
          TIL.Connectors.VLEFluidPort portA_vle( final vleFluidType=vleFluidType)
            "VLEFluid portA"
            annotation (Placement(transformation(extent={{-150,-10},{-130,10}},
                  rotation=0)));
          TIL.Connectors.GasPort portA_gas( final gasType=gasType) "Gas portA"
            annotation (Placement(transformation(extent={{-10,130},{10,150}}, rotation=
                    0)));
          TIL.Connectors.GasPort portB_gas( final gasType=gasType) "Gas portB"
            annotation (Placement(transformation(extent={{-10,-150},{10,-130}},
                  rotation=0)));

          /****************** Splitter and joiner *******************/

        protected
          SplitterJoiner.GasSplitterJoiner gasSplitterJoiner(
            final nPorts1 = nCells,
            final nPorts2 = 1,
            final gasType=gasType,
            final cellFlowType=moistAirCellFlowType)
            annotation (Placement(transformation(extent={{-26,-46},{26,-26}})));

        /********* Summary ***************/

        protected
          parameter Boolean includeSummaryArrays = true
            "Obsolete & unused parameter for array entries in summary"
         annotation(Dialog(tab="Advanced", group="Summary"));

          record CrossSummary
            extends Summary;

            replaceable Arrays arrays;

          protected
            record Arrays
              parameter Integer n;
              parameter Integer m;

              input SI.Temperature[n + 1] T_vle_port "Temperature at port";
              input SI.Temp_C[n + 1] T_degC_vle_port "Temperature at port";
              input SI.Temperature[n] T_vle_cell "Temperature in cell";
              input SI.Temp_C[n] T_degC_vle_cell "Temperature in cell";
              input SI.MassFraction[n + 1] q_vle_port
                "Vapor quality (steam mass fraction) at port";
              input SI.Pressure[n + 1] p_vle_port "Pressure at port";
              input SI.SpecificEnthalpy[n + 1] h_vle_port "Specific enthalpy at port";
              input SI.Density[n + 1] d_vle_port "Density at port";
              input SI.MassFlowRate[n + 1] m_flow_vle_port "Mass flow rate at port";
              input SI.Velocity[n + 1] w_vle_port "Flow velocity at port";
              input SI.CoefficientOfHeatTransfer[n] alpha_vle_cell
                "Heat transfer coefficient";

              input SI.Temperature[n] T_finSideWallSurface_cell "Temperature in cell";
              input SI.Temp_C[n] T_degC_finSideWallSurface_cell "Temperature in cell";
              input SI.Temperature[n] T_wall_cell "Temperature in cell";
              input SI.Temp_C[n] T_degC_wall_cell "Temperature in cell";

              input SI.Temperature[n] T_air_portA "Temperature at port A";
              input SI.Temp_C[n] T_degC_air_portA "Temperature at port A";
              input SI.Temperature[n] T_air_portB "Temperature at port B";
              input SI.Temp_C[n] T_degC_air_portB "Temperature at port B";
              input SI.Pressure[n] p_air_portA "Pressure at port A";
              input SI.Pressure[n] p_air_portB "Pressure at port B";
              input SI.SpecificEnthalpy[n] h_air_portA "Specific enthalpy at port A";
              input SI.SpecificEnthalpy[n] h_air_portB "Specific enthalpy at port B";
              input SI.MassFraction[m, n] xi_air_portA "Air mass fraction at port A";
              input SI.MassFraction[m, n] xi_air_portB "Air mass fraction at port B";
              input SI.MassFraction[n] xis_air_wall_cell
                "Saturation mass fraction of condensing component in cell";
              input Real[n] phi_air_portA "Relative humidity at port A";
              input Real[n] phi_air_portB "Relative humidity at port B";
              input SI.CoefficientOfHeatTransfer[n] alpha_air_cell
                "Heat transfer coefficient";

            end Arrays;
          end CrossSummary;

        protected
          TILMedia.Gas_ph[nCells] moistAir_outflow(
            final p = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAirCell[i].moistAirPortB.p else moistAirCell[i].moistAirPortA.p for i in 1:nCells},
            final h = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAirCell[i].moistAirPortB.h_outflow else moistAirCell[i].moistAirPortA.h_outflow for i in 1:nCells},
            final xi = {if ((moistAirCellFlowType == "flow A-B") or ((moistAirCellFlowType <> "flow B-A") and noEvent(moistAirCell[i].moistAirPortA.m_flow >= 0))) then moistAirCell[i].moistAirPortB.xi_outflow else moistAirCell[i].moistAirPortA.xi_outflow for i in 1:nCells},
            each final gasType=gasType,
            each computeTransportProperties=false) if
                                           includeDefaultSummary
            annotation (Placement(transformation(extent={{-50,-100},{-30,-80}},rotation=
                   0)));
        equation
        // Connect wall and vleFluid cells with eachother
          for i in 1:nCells-1 loop
            connect(vleFluidCell[i].portB, vleFluidCell[i+1].portA);
            connect(moistAirCell[i].wallHeatPortE, moistAirCell[i+1].wallHeatPortW);
          end for;

          connect(vleFluidCell[nCells].portB, portB_vle)
            annotation (Line(
              points={{10,30},{100,30},{100,0},{140,0}},
              color={153,204,0},
              thickness=0.5));

          connect(portA_vle, vleFluidCell[1].portA) annotation (Line(
              points={{-140,0},{-100,0},{-100,30},{-10,30}},
              color={153,204,0},
              thickness=0.5));
          connect(vleFluidCell.heatPort, moistAirCell.wallHeatPortN) annotation (Line(
              points={{0,20},{0,-20}},
              color={204,0,0},
              pattern=LinePattern.Solid,
              thickness=0.5));

          connect(gasSplitterJoiner.outlet, portB_gas) annotation (Line(
              points={{26,-36},{40,-36},{40,-100},{0,-100},{0,-140}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          connect(gasSplitterJoiner.inlet, portA_gas) annotation (Line(
              points={{-26,-36},{-40,-36},{-40,100},{0,100},{0,140}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          connect(gasSplitterJoiner.outlets[:, 1], moistAirCell.moistAirPortA)
            annotation (Line(
              points={{-14,-36},{-10,-36}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          connect(gasSplitterJoiner.inlets[:, 1], moistAirCell.moistAirPortB)
            annotation (Line(
              points={{14,-36},{10,-36}},
              color={255,153,0},
              thickness=0.5,
              smooth=Smooth.None));
          annotation(Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td colspan=2>VLE fluid</td>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient (default) or steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>h, xi, alphaAState, pressureDropState (default) or derivative of pressure in the PressureStateElement</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure drop or isobaric</td>
        </tr>
        <tr>
        <td colspan=2>Wall</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>T</td>
        </tr>
        <tr>
        <td colspan=2>Moist air</td>
        </tr>
        <tr>
        <td>mass balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>transient</td>
        </tr>
        <tr>
        <td>differential states:</td><td>H_WallPlusFilm, massFilm, pressureDropState</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure drop or isobaric</td>
        </tr>
        </table>
<p>
This is the base model for a cross flow heat exchanger. The following figure illustrates
the flow situation.<br><br>
</p>
<img src=\"modelica://TIL/Images/TIL3_InfoView_HXStructure_CrossFlow.png\" width=\"600\">
</html>"),  Diagram(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-140,-140},{140,140}},
                initialScale=0.1), graphics),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-140,-140},{140,140}},
                initialScale=0.1), graphics={
                Bitmap(extent={{-140,-140},{140,140}},
                  imageSource=
                      "iVBORw0KGgoAAAANSUhEUgAAARgAAAEYCAIAAAAI7H7bAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAACHlJREFUeNrs3U+LVWUcwHGnojAMA6NQDIXCCIqM2thGZ1ObRKtVq6ZNLbNXkL2CpmVushdQGbXJzYwb2yQVRpEkjBSGkWAYhVFMP7o0Hc9z751z/5xzn3Pu54NEc51m7rnn+T7nnOf+acsWAAAA/rNQ/GJlZcUjkpePF///98P2Tl4WFxf7h7S+vu7RycuJwg56xd7J7Ci08P/eucXDAZMTEggJhARCAoQEQgIhgZAAIYGQQEggJEBIICQQEggJEBIICYQEQgKEBEICIYGQACGBkEBIgJBASCAkEBIgJBASCAmEBAgJhARCAiEBQgIhgZBASICQQEggJBASICQQEggJhAQICYQEQgKEBEICIYGQACGBkEBIICRASCAkEBIICRASCAmEBEIChARCAiGBkAAhgZBASCAkDwEICYQEQgKEBEICIYGQACGBkEBIICRASCAkEBIICRASCAmEBEIChARCAiGBkAAhgZBASICQQEggJBASULRQ/GJ9fd0jAlXjWVhwRAKndiAkEBLQrZDWrp365fcvO7+HLl9fjT+d38zYlbFDhdS0sz8cO33xuU8uHOr2IPvul5OfXFiMP/Ev3Z4sYlfGDo3d2tJNaOXy9+ra0oWr7218eXDPuw/ds9S94XXu8vFzP7258eUTO994YtfxTk4WZy69vPHlvh0vHdrbjlmjuPzdspBu/HXt9MWjP/12pnR79wZZabJo3SCr6PyV5c9+fL10485tB59+4NQdt90tpLoqihOAq3981fdvOzPIBk0W7RpkY08WPTu2PvbsvtXMN7OVIcXFaFT059+/btxy+63bi1+GPduPREutHmTpZBGbGf8sbmkrBtmmmxkVXfr1o+KNpR0aX8Zm3nPn/laE1I7Fht7FaDqY4uqo+G2xY+LbYie1dHjFZPH+t/tLFcVmxp9eTj3xDfFt7V2x7E0WpYpiV8Zmxm7duCV2d4sWk1pwRCpdjJam5PRIte32PXHyk/NMVvGQW9yQ+Ns43/vtz0stmrAHbeaQDel79p7tYlKbTu3Si9H0/K0Dg2z4ZDHkErFdK5ZVZr2+Z30Hdr/16H3HhDS1i9FBKwqtHmRVJovWDbLxJosxdr2QRr4YHT5i+i525d/SGCOmFYNs04o2XX6sPr8IadjF6HiHlxYNstjMz348Vrq3FQ8v6SCLzTyweznPpbyxd8pIBzEhlU+jz6wtpStX1S94Si8IyHOQTX4umvkgGzJZjPTseXpZFZt5cO/JHC6A8w2p75NFYywbZD7I+k4WcZ6z665D5W/9uHDL4dXSX16+vhpns3kOsileuE5rVMxLSDH6Y/aa1jOP2Q6y0YbFicIOemW9RYNshMlirCbjp8VZxmwvgHMMaYyL0TbOZCNPFpuFlOcgq+ORz3AxKbtXNsTFaKmiuKo5/NCkZ2Kx2154+Mv0yfJZve+lN1kUh1dMFpOfcMZ/Hj8kflRxM+MXzeqdF/Hwplc1sSMmnL9iM2NIxMAo3hibGYMnhzF8Sw4VpRej01pnu+uOvekLT05ffK75QXb2h2N1TBbDB1nzb++JBzYe3vSQGztiKj8/BkYMj+ItMXhyaOmmU7t3Pp992TUdrNNcH7n3tafuX57VZFH1t1c4tSvl+vXPb5dybWz1v7Hfnl4IzMSrT+Z0RLr5YvTDmk55Y3fG2C3eEru8gZkszuzf/2Z/qaKYLGpqOH5s6YW88avjDjTwQt54MEsVxQNeU8MxSGKoFF/I69TupovRvXcfre9XND/Irt9YS5cB6pssBg2yuANxN+LOdGOy6ImhUnpRvJCmczE6ySCro6VB74modbIYNMjqe+dF3zXDZhbT0sWkXK6R5uQjixtYFu/7LNbTD5wa+Zp7xGuk0vEw7sO0nsmZ1SOZs3n/yOLYzbW+h6z30T/1rVxV1HfFcoofSDTo3ZZzUlGm10h5tDSFQXbu8vF0mXtWr07qPcWULovHnaxpspjPirbM8yet1jHIVteW0tfLzvaV//Gr4w6UNjPu5CQrlllNFrmc5s3hNVI6+id/50VdL2CZ4BopPYZM5UVYbXw3lGukJsQgSJfFP/3+aPWlvN7KVbGixlauqos7E3epuJQXd3ikFcv4znhY0jdQzWdFjkhVJ+yKrz2v9xMjpndE2ri3431WTAc+McIRqaEJ+/mHvxj1uZfeuCxWFOMy52vu3ipL3MmNW+LOxyZsupnpc2LxcM1zRRYbpjbI4iD2wbePT/1lzg1sZvqi+NiQQSuWrZsshNSmQXb+ynLpVHDP9iNtWbnqrVjGHS7eGJsTG9WNyUJI7Rhkq2tL6WePPPNgmz6SO+5q3OHSsnhsVHFZPL1ubNFkYbEhF33XeQ/sXh77o39yWGxIDfrUq3Qz53aZu8pig5CGST+QKP3k/npXruoPqe+RJ93Mrv7fmYTUkCHvIZv6a0BnFdKWfq+ybW6y6ERIrpE2EQPo2X0r6fteek8W1VtRg2JD+r6959/NXFGRxYZaBtmOrY+9+Mhax1auYnNio4orlh2bLISU1yCbykf/5Kn4gUSdnCxqPM1zjVTdjb+uXbh6stH/9UNT10gl568s79uxZJnbYkNXzCgkLDaAayQQEggJEBIICYQECAmEBEICIQFCAiGBkEBIgJBASCAkEBIgJBASCAmEBAgJhARCAiEBQgIhgZBASICQQEggJEBIICQQEggJEBIICYQEQgKEBEICIYGQACGBkEBIICRASCAkEBIICRASCAmEBEIChARCAiEBQgIhgZBASICQQEggJBASICQQEggJhAQICYQEQgIhAUICIYGQQEiAkEBIICRASCAkEBIICRASCAmEBEIChARCAiGBkAAhgZBASCAkQEggJBASCAkQEggJhARCAoQEQgIhAUICIYGQQEgAAAAAA/wjwAAuFnEcIolTZQAAAABJRU5ErkJggg==",
                  fileName="modelica://TIL/Images/CrossFlowHX_VLEGas.png"),
                Text(
                  extent={{-46,110},{194,70}},
                  lineColor={153,204,0},
                  textString=
                       "(%pressureStateID)"),
                Text(
                  extent={{-140,0},{-100,-40}},
                  lineColor={0,0,0},
                  textString=
                       "1"),
                Text(
                  extent={{100,0},{140,-40}},
                  lineColor={0,0,0},
                  textString=
                       "n"),
                Text(
                  extent={{-132,130},{-92,90}},
                  lineColor={0,0,0},
                  textString=
                       "A"),
                Text(
                  extent={{92,-92},{132,-132}},
                  lineColor={0,0,0},
                  textString=
                       "B")}),
                        points=[-140,0; -140,0], style(color=0, rgbcolor={0,0,0}),
            Line(points={{-140,0},{-140,0}}, color={0,0,0}));
        end CrossFlowHX;

        package BaseClasses
        extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialHX "Partial base class for fin-and-tube heat exchangers"

          /*********************** SIM *************************/

            parameter TILMedia.GasTypes.BaseGas           gasType = sim.gasType1
              "Gas type" annotation (Dialog(tab="SIM",group="SIM"),choices(
              choice=sim.gasType1 "Gas 1 as defined in SIM",
              choice=sim.gasType2 "Gas 2 as defined in SIM",
              choice=sim.gasType3 "Gas 3 as defined in SIM"));
            parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType = sim.vleFluidType1
              "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
              choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
              choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
              choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));
          protected
            outer SystemInformationManager sim "System information manager";
            TIL.Internals.SimPort simPort;

          /*********** Connectors *************************/
          public
            TIL.Connectors.VLEFluidPort portB_vle(final vleFluidType=vleFluidType);
            TIL.Connectors.VLEFluidPort portA_vle(final vleFluidType=vleFluidType);
            TIL.Connectors.GasPort portA_gas(final gasType=gasType);
            TIL.Connectors.GasPort portB_gas(final gasType=gasType);

          /****************** Geometry, Discretization *******************/

            inner replaceable parameter TIL.HeatExchangers.FinAndTube.Geometry.KKI001
              hxGeometry
              constrainedby
              TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              "Geometry of heat exchanger"
              annotation(Dialog(group="General"),choicesAllMatching,
                Placement(transformation(extent={{-100,100},{-80,120}})));

           parameter Integer nCells(min=1)=1 "Discretization number of cells"
              annotation (Dialog(group="General"));

            parameter Integer nFinSideParallelHydraulicFlows(min=1)
              "Sum of entering hydraulic flows for all cells"
              annotation (Dialog(tab="Discretization",group="Fin Side"));
            parameter Integer nFinSideParallelHydraulicFlowsPerCell(min=1)
              "Number of hydraulic flows for a single cell"
              annotation (Dialog(tab="Discretization",group="Fin Side"));
            parameter Integer nTubeSideParallelHydraulicFlowsPerCell(min=1)
              "Number of hydraulic flows for a single cell"
              annotation (Dialog(tab="Discretization",group="Tube Side"));

          protected
            TIL.Cells.Geometry.FluidCellGeometry vleFluidCellGeometry(
              length=hxGeometry.tubeSidePathLength/nCells,
              volume=hxGeometry.totalTubeInnerVolume/nCells,
              heatTransferArea=hxGeometry.totalTubeSideHeatTransferArea/nCells,
              hydraulicCrossSectionalArea=PI*hxGeometry.tubeInnerDiameter*hxGeometry.tubeInnerDiameter/4.0,
              nParallelHydraulicFlows= nTubeSideParallelHydraulicFlowsPerCell);

            TIL.Cells.Geometry.WallCellGeometry wallCellGeometry(
              length=hxGeometry.tubeSidePathLength/nCells,
              volume=(hxGeometry.totalTubeOuterVolume-hxGeometry.totalTubeInnerVolume)/nCells);

            TIL.Cells.Geometry.FluidCellGeometry moistAirCellGeometry(
              volume=hxGeometry.totalFinnedVolume*hxGeometry.voidRatio/nCells,
              heatTransferArea=hxGeometry.totalFinSideHeatTransferArea/nCells,
              hydraulicCrossSectionalArea=hxGeometry.totalFinSideCrossSectionalArea*hxGeometry.areaRatio/nFinSideParallelHydraulicFlows,
              finHeatTransferAreaRatio=hxGeometry.finSideHeatTransferAreaRatio,
              nParallelHydraulicFlows=nFinSideParallelHydraulicFlowsPerCell);

          /****************** VLEFluid *******************/

          public
            parameter TIL.Internals.PressureStateID pressureStateID=1 "Pressure state ID"
              annotation(Dialog(tab="SIM",group="SIM"));

            SI.Mass cumulatedVLEFluidMass=sum(vleFluidCell.mass)
              "Cumulated vleFluid mass";
            SI.Volume cumulatedVLEFluidVolume=sum(vleFluidCell.cellGeometry.volume)
              "Cumulated vleFluid volume";

          protected
            Cells.VLEFluidCell[nCells] vleFluidCell(
              each final cellFlowType = vleFluidCellFlowType,
              each cellGeometry = vleFluidCellGeometry,
              each final dpdt=sim.dpdtPort[pressureStateID].dpdt,
              each final vleFluidType=vleFluidType,
              each final removeSingularity=sim.removeSingularity,
              each final generateEventsAtFlowReversal=sim.generateEventsAtFlowReversal,
              redeclare each final model HeatTransferModel =
                  TubeSideHeatTransferModel,
              redeclare each final model PressureDropModel =
                  TubeSidePressureDropModel,
              each final pStart=pVLEFluidStart,
              each final m_flowStart=m_flowVLEFluidStart,
              each final orientation=cellOrientation,
              each final useFalseDynamics = useFalseDynamics,
              each final pressureDropInitial = pressureDropInitialVLEFluid/nCells,
              each final fixedPressureDropInitial = fixedPressureDropInitialVLEFluid,
              each final falseDynamicsTimeConstant = falseDynamicsTimeConstant,
              each final alphaAInitial= if (initHeatTransfer=="alphaA") then alphaAInitialVLEFluid/nCells
                                         else alphaInitialVLEFluid*hxGeometry.totalTubeSideHeatTransferArea/nCells,
              each final alphaAStateTimeConstant = alphaAStateTimeConstant,
              each useAlphaAState=if (TIL.Internals.AlphaAStateChoice.useAlphaAState ==
                  alphaAStateChoice) then true elseif (TIL.Internals.AlphaAStateChoice.doNotUseAlphaAState
                   == alphaAStateChoice) then false else vleFluidCell[1].heatTransfer.useAlphaAState)
              "VLEFluid cells"
              annotation (Placement(transformation(extent={{-10,20},{10,40}}, rotation=0)));

          public
            replaceable model TubeSideHeatTransferModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ConstantAlpha
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.PartialVLEFluidHeatTransfer
              "Heat transfer model"
              annotation(Dialog(group="VLEFluid"),choicesAllMatching);

            replaceable model TubeSidePressureDropModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.PartialVLEFluidPressureDrop
              "VLEFluid pressure drop model"
              annotation(Dialog(group="VLEFluid"),choicesAllMatching);

          protected
            parameter TIL.Internals.CellFlowType vleFluidCellFlowType = "allow reverse flow" annotation(Dialog(tab="Advanced",group="VLEFluid"));

          public
            parameter TIL.Cells.Internals.VLEFluidCellOrientationType cellOrientation= "A"
              "selection of cell orientation for equations of mDotHydraulic and pressure"
                 annotation(Dialog(tab="Advanced",group="VLEFluid Cells"));

            parameter Boolean useFalseDynamics=true
              "sets pressure drop as differential state; True, if false dynamics should be used"
              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

            parameter Real falseDynamicsTimeConstant=0.1
              "Time constant of false dynamics for pressureDropState"
              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

            parameter TIL.Internals.AlphaAStateChoice
                      alphaAStateChoice = TIL.Internals.AlphaAStateChoice.definedByHeatTransferCorrelation
              "choice 1: force use; choice 2: force use not; choice 3: defined by heat transfer"
              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

            parameter Real alphaAStateTimeConstant=1 "Time constant for alphaAState"
                                              annotation(Dialog(group="VLEFluid Cells", tab="Advanced"));

          /****************** Wall *******************/

             parameter TIL.Cells.Internals.WallCellStateType wallCellStateType= "state center"
              "position of differential state T in wall cells"                                                                                  annotation(Dialog(tab="Advanced",group="Wall Cells"));

            replaceable model WallMaterial =
                TILMedia.SolidTypes.TILMedia_Aluminum
              constrainedby TILMedia.SolidTypes.BaseSolid "Wall material"
            annotation(Dialog(group="Tube"),choicesAllMatching=true);

            replaceable model WallHeatConductionModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.GeometryBasedConduction
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.PartialHeatTransfer
              "Heat transfer model"
              annotation(Dialog(group="Tube"),choicesAllMatching=true);

          /****************** Fin *******************/

            replaceable model FinMaterial =
                TILMedia.SolidTypes.TILMedia_Aluminum
              constrainedby TILMedia.SolidTypes.BaseSolid "Solid fluid type"
            annotation(Dialog(group="Fin"),choicesAllMatching=true);

            replaceable model FinEfficiencyModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.Schmidt
                constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.PartialFinEfficiency
              "Fin efficiency model" annotation(Dialog(group="Fin"),choicesAllMatching=true);

          /****************** Moist Air *******************/

          protected
            TIL.Cells.MoistAirWallCell[nCells] moistAirCell(
              each cellGeometry = moistAirCellGeometry,
              redeclare each final model MoistAirHeatTransferModel =
                  FinSideHeatTransferModel,
              redeclare each final model MoistAirPressureDropModel =
                  FinSidePressureDropModel,
              redeclare final model FinEfficiencyModel = FinEfficiencyModel,
              each final gasType=gasType,
              each final m_flowStart=m_flowMoistAirStart,
              each final massFilmInitial=mWaterInitial/nCells,
              each final massFilmMax=mWaterMax/nCells,
              each final dryOutRatio = dryOutRatio,
              each final cellFlowType=moistAirCellFlowType,
              redeclare each final model FinMaterial = FinMaterial,
              each TWallInitial=TInitialWall,
              redeclare each final model WallMaterial =
                                          WallMaterial,
              redeclare each final model WallHeatTransferModel =
                  WallHeatConductionModel,
              each wallCellStateType = wallCellStateType,
              each final flagActivateDynWaterBalance = flagActivateDynWaterBalance,
              each final flagActivateWallCondensation = flagActivateWallCondensation,
              each final wallCellGeometry = wallCellGeometry,
              each final useFalseDynamics = useFalseDynamicsMoistAir,
              each final falseDynamicsTimeConstant = falseDynamicsTimeConstantMoistAir,
              each final pressureDropMoistAirInitial = pressureDropInitialMoistAir,
              each final fixedPressureDropMoistAirInitial = fixedPressureDropInitialMoistAir)
              "MoistAir cells"
              annotation (Placement(transformation(extent={{-10,-40},{10,-20}}, rotation=
                      0)));

          public
            replaceable model FinSideHeatTransferModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.ConstantAlpha
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.PartialHeatTransfer
              "Heat transfer model"
                annotation(Dialog(group="Moist Air - (see also Advanced Moisture Handling)"),choicesAllMatching=true);

            replaceable model FinSidePressureDropModel =
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSidePressureDrop.ZeroPressureDrop
              constrainedby
              TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSidePressureDrop.PartialPressureDrop
              "Pressure drop model"
                annotation(Dialog(group="Moist Air - (see also Advanced Moisture Handling)"),choicesAllMatching=true);

            parameter TIL.Internals.CellFlowType moistAirCellFlowType = "flow A-B"
              "allowed flow directions in moist air cell"
              annotation(Dialog(tab="Advanced",group="Moist Air Cells"));

            parameter Boolean useFalseDynamicsMoistAir=true
              "sets pressure drop as differential state; True, if false dynamics should be used"
              annotation(Dialog(group="Moist Air Cells", tab="Advanced"));

            parameter Real falseDynamicsTimeConstantMoistAir=0.1
              "Time constant of false dynamics for pressureDropState"
              annotation(Dialog(group="Moist Air Cells", tab="Advanced"));

            parameter Boolean flagActivateWallCondensation=true
              "if true, water condenses on the wall surface" annotation(Dialog(tab="Advanced", group="Moisture Handling"));
            parameter Boolean flagActivateDynWaterBalance=false
              "if true, the condensate on the wall surface is balanced dynamically" annotation(Dialog(tab="Advanced", group="Moisture Handling"));
            parameter SI.Mass mWaterMax=0.02 "Maximum water mass in total HX" annotation(Dialog(enable=(flagActivateDynWaterBalance), tab="Advanced", group="Moisture Handling"));
            parameter Real dryOutRatio = 0.6
              "below this water ratio (mWater/mWaterMax), wetting is partial" annotation(Dialog(enable=(flagActivateDynWaterBalance), tab="Advanced", group="Moisture Handling"));

            /****************** Start values *******************/

            parameter SI.MassFlowRate m_flowVLEFluidStart=0.05
              "Start value for mass flow rate"
              annotation(Dialog(tab="Start Values",group="VLEFluid"));
            parameter SI.MassFlowRate m_flowMoistAirStart = 0.05
              "Start value for moist air mass flow rate"
              annotation(Dialog(tab="Start Values",group="MoistAir"));

            parameter SI.AbsolutePressure pVLEFluidStart=1e5 "Start value for pressure"
              annotation(Dialog(tab="Start Values",group="VLEFluid"));

            /****************** Initialization *******************/

            inner parameter TIL.Internals.InitializationMethods initVLEFluid=
                                                                    "constantEnthalpy"
              "Initialization method"
              annotation (Dialog(tab="Initialization",group="VLEFluid"),choicesAllMatching=true);

          // Values for constant enthalpy
            inner parameter SI.SpecificEnthalpy hInitialVLEFluid=300e3
              "Initial value for specific enthalpy"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="constantEnthalpy")));

          // Values for linear enthalpy profile
            parameter SI.SpecificEnthalpy hInitialVLEFluid_Cell1=250e3
              "Initial value for specific enthalpy of cell 1"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="linearEnthalpyDistribution")));
            parameter SI.SpecificEnthalpy hInitialVLEFluid_CellN=350e3
              "Initial value for specific enthalpy of cell n"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="linearEnthalpyDistribution")));

          // Values for constant temperature
            parameter SI.Temperature TInitialVLEFluid=298.15
              "Initial value for temperature"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="constantTemperature")));

          // Values for user specific specific enthalpies
            parameter SI.SpecificEnthalpy[nCells] hInitialUserValues=ones(nCells)*hInitialVLEFluid
              "Initial values for specific enthalpies"
              annotation(Dialog(tab="Initialization",group="VLEFluid",enable=(initVLEFluid=="userSpecifiedValues")));

            parameter SI.Pressure pressureDropInitialVLEFluid = 0
              "Initial value for pressure drop"
              annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization"));

            parameter Boolean fixedPressureDropInitialVLEFluid = true
              "if true, force usage of initial value" annotation(Dialog(tab="Advanced Initialization", group="VLEFluid"));

            parameter SI.Temperature TInitialWall=298.15 "Initial value for temperature"
              annotation (Dialog(tab="Initialization",group="Wall"));

            parameter SI.Pressure pressureDropInitialMoistAir = 0
              "Initial value for pressure drop"
              annotation(Dialog(group = "Moist Air",tab="Advanced Initialization"));

            parameter Boolean fixedPressureDropInitialMoistAir = true
              "if true, force usage of initial value" annotation(Dialog(tab="Advanced Initialization", group="Moist Air"));

            parameter SI.Mass mWaterInitial=0
              "Initial water mass in cell- This should be less or equal than mWaterMax" annotation(Dialog(enable=(flagActivateDynWaterBalance), tab="Advanced Initialization", group="Moist Air"));

            parameter String initHeatTransfer="alphaA"
              "Choice of initialization variable"
              annotation(choices(
                  choice= "alpha" "use alphaInitial for initialization",
                  choice= "alphaA" "use alphaAInitial for initialization"),
                  Dialog(tab="Advanced Initialization", group="VLEFluid"));

            parameter SI.CoefficientOfHeatTransfer alphaInitialVLEFluid = 0
              "Initial value for alpha"
              annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization",enable=(initHeatTransfer=="alpha")));

            parameter Real alphaAInitialVLEFluid = 0 "Initial value for alphaA"
              annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization",enable=(initHeatTransfer=="alphaA")));

            parameter Real[vleFluidType.nc] mixingRatioInitialVLEFluid = vleFluidType.defaultMixingRatio
              "Initial value for mixing ratio" annotation(Dialog(group = "VLEFluid",tab="Advanced Initialization"));

            /****************** Additional variables *******************/
          protected
            TILMedia.Gas_ph   moistAirA(
              final gasType=gasType,
              final p = portA_gas.p,
              final h = noEvent(actualStream(portA_gas.h_outflow)),
              final xi= noEvent(actualStream(portA_gas.xi_outflow)),
              computeTransportProperties=false) if                      includePortVLEFluid
              "Moist air at inlet" annotation (Placement(transformation(extent={{-80,-70},
                      {-60,-50}}, rotation=0)));
            TILMedia.Gas_ph   moistAirB(
              final gasType=gasType,
              final p = portB_gas.p,
              final h = noEvent(actualStream(portB_gas.h_outflow)),
              final xi = noEvent(actualStream(portB_gas.xi_outflow)),
              computeTransportProperties=false) if                       includePortVLEFluid
              "Moist air at outlet"  annotation (Placement(
                  transformation(extent={{60,-70},{80,-50}}, rotation=0)));

            TILMedia.VLEFluid_ph    vleFluidA(
              final vleFluidType = vleFluidType,
              final p=portA_vle.p,
              final h=noEvent(actualStream(portA_vle.h_outflow)),
              final xi={noEvent(actualStream(portA_vle.xi_outflow[i])) for i in 1:
                  vleFluidType.nc - 1}) if includePortVLEFluid "Outlet vleFluid"
              annotation (Placement(transformation(extent={{-80,50},{-60,70}}, rotation=0)));

            TILMedia.VLEFluid_ph    vleFluidB(
              final vleFluidType=vleFluidType,
              final p=portB_vle.p,
              final h=noEvent(actualStream(portB_vle.h_outflow)),
              final xi={noEvent(actualStream(portB_vle.xi_outflow[i])) for i in 1:
                  vleFluidType.nc - 1}) if includePortVLEFluid "Outlet vleFluid"
              annotation (Placement(transformation(extent={{60,50},{80,70}}, rotation=0)));

            /*******************Summary**********************/
          public
            parameter Boolean includeDefaultSummary = true
              "Include summary record in model results"
           annotation(Dialog(tab="Advanced", group="Summary"));

          protected
            parameter Boolean includePortVLEFluid = includeDefaultSummary;

          protected
            record Summary
              extends TIL.Internals.ClassTypes.Record;

              SI.Pressure p_vle_A "Pressure at port A";
              SI.Pressure p_vle_B "Pressure at port B";
              SI.Temperature T_vle_A "Temperature at port A";
              SI.Temperature T_vle_B "Temperature at port B";
              SI.Temp_C T_degC_vle_A "Temperature at port A";
              SI.Temp_C T_degC_vle_B "Temperature at port B";
              SI.MassFraction q_vle_A "Vapor quality (steam mass fraction) at port A";
              SI.MassFraction q_vle_B "Vapor quality (steam mass fraction) at port B";
              SI.SpecificEnthalpy h_vle_A "Specific enthalpy at port A";
              SI.SpecificEnthalpy h_vle_B "Specific enthalpy at port B";
              SI.Density d_vle_A "Density at port A";
              SI.Density d_vle_B "Density at port B";
              SI.MassFlowRate m_flow_vle_A "Mass flow rate at port A";
              SI.MassFlowRate m_flow_vle_B "Mass flow rate at port B";
              SI.Pressure dp_vle "Total pressure drop";
              SI.HeatFlowRate Q_flow_vle "Total heat flow rate";
              SI.TemperatureDifference superheating "Superheating";
              SI.TemperatureDifference subcooling "Subcooling";
              SI.Mass mass_vle "Total fluid mass";
              SI.Volume volume_vle "Total fluid volume";
              SI.CoefficientOfHeatTransfer alpha_average_vle
                "Average coefficient of heat transfer";
              SI.ThermalConductance alphaA_average_vle
                "Average coefficient of thermal conductance";

              SI.Pressure p_air_A "Pressure at port A";
              SI.Pressure p_air_B "Pressure at port B";
              SI.Temperature T_air_A "Temperature at port A";
              SI.Temperature T_air_B "Temperature at port B";
              SI.Temp_C T_degC_air_A "Temperature at port A";
              SI.Temp_C T_degC_air_B "Temperature at port B";
              SI.SpecificEnthalpy h_air_A "Specific enthalpy at port A";
              SI.SpecificEnthalpy h_air_B "Specific enthalpy at port B";
              SI.MassFlowRate m_flow_air_A "Mass flow rate at port A";
              SI.MassFlowRate m_flow_air_B "Mass flow rate at port B";
              SI.MassFraction humRatio_air_A
                "Content of condensing component aka humidity ratio at port A";
              SI.MassFraction humRatio_air_B
                "Content of condensing component aka humidity ratio at port B";
              Real phi_air_A(final unit="1", min=0, max=100)
                "Relative humidity at port A";
              Real phi_air_B(final unit="1", min=0, max=100)
                "Relative humidity at port B";
              SI.Velocity w_air_A "Flow velocity at port A";
              SI.Velocity w_air_B "Flow velocity at port B";
              SI.Pressure dp_air "Total pressure drop";
              SI.MassFlowRate m_flow_water_latent "Latent water mass flow rate";
              SI.MassFlowRate m_flow_water_liquid "Liquid water mass flow rate";
              SI.Mass mass_water "Mass of water";
              SI.HeatFlowRate Q_flow_air "Total heat flow rate";
              SI.HeatFlowRate Q_flow_air_sensible "Sensible heat flow rate";
              SI.HeatFlowRate Q_flow_air_latent "Latent heat flow rate";
              SI.CoefficientOfHeatTransfer alpha_average_air
                "Average coefficient of heat transfer";
              SI.ThermalConductance alphaA_average_air
                "Average coefficient of thermal conductance";

              SI.Mass mass_component "Solid mass of component";

              parameter Geometry.Summary geometry;

            end Summary;

            replaceable record SummaryClass = Summary;

          public
            SummaryClass summary(
              p_vle_A = vleFluidA.p,
              p_vle_B = vleFluidB.p,
              T_vle_A = vleFluidA.T,
              T_vle_B = vleFluidB.T,
              T_degC_vle_A = Modelica.SIunits.Conversions.to_degC(vleFluidA.T),
              T_degC_vle_B = Modelica.SIunits.Conversions.to_degC(vleFluidB.T),
              q_vle_A = vleFluidA.q,
              q_vle_B = vleFluidB.q,
              h_vle_A = vleFluidA.h,
              h_vle_B = vleFluidB.h,
              d_vle_A = vleFluidA.d,
              d_vle_B = vleFluidB.d,
              m_flow_vle_A = portA_vle.m_flow,
              m_flow_vle_B = portB_vle.m_flow,
              dp_vle = vleFluidA.p - vleFluidB.p,
              Q_flow_vle = sum(vleFluidCell.heatPort.Q_flow),
              superheating = noEvent(max(0, noEvent(max(vleFluidB.T -vleFluidB.VLE.T_v, vleFluidA.T -
                  vleFluidA.VLE.T_v)))),
              subcooling = noEvent(max(0, noEvent(max(vleFluidB.VLE.T_l- vleFluidB.T,
                  vleFluidA.VLE.T_l                                                                          - vleFluidA.T)))),
              mass_vle = cumulatedVLEFluidMass,
              volume_vle = cumulatedVLEFluidVolume,
              alpha_average_vle = sum(vleFluidCell.alphaAState)/hxGeometry.totalTubeSideHeatTransferArea,
              alphaA_average_vle = sum(vleFluidCell.alphaAState),
              p_air_A = moistAirA.p,
              p_air_B = moistAirB.p,
              T_air_A = moistAirA.T,
              T_air_B = moistAirB.T,
              T_degC_air_A = Modelica.SIunits.Conversions.to_degC(moistAirA.T),
              T_degC_air_B = Modelica.SIunits.Conversions.to_degC(moistAirB.T),
              h_air_A = moistAirA.h,
              h_air_B = moistAirB.h,
              m_flow_air_A = portA_gas.m_flow,
              m_flow_air_B = portB_gas.m_flow,
              humRatio_air_A=moistAirA.humRatio,
              humRatio_air_B=moistAirB.humRatio,
              phi_air_A = moistAirA.phi,
              phi_air_B = moistAirB.phi,
              w_air_A = portA_gas.m_flow/moistAirA.d/hxGeometry.totalFinSideCrossSectionalArea,
              w_air_B = -portB_gas.m_flow/moistAirB.d/hxGeometry.totalFinSideCrossSectionalArea,
              dp_air = moistAirA.p - moistAirB.p,
              m_flow_water_latent = sum(moistAirCell.mdotEvaporate) - sum(moistAirCell.mdotCondensate),
              m_flow_water_liquid= sum(moistAirCell.mdotWaterDrain),
              mass_water = sum(moistAirCell.massFilm),
              Q_flow_air = -sum(moistAirCell.QflowMoistAirToWall),
              Q_flow_air_sensible = -(sum(moistAirCell.QflowMoistAirToWall)-sum(moistAirCell.QflowMoistAirToWallLatent)),
              Q_flow_air_latent = -sum(moistAirCell.QflowMoistAirToWallLatent),
              alpha_average_air = sum(moistAirCell.heatTransfer.alphaA)/hxGeometry.totalFinSideHeatTransferArea,
              alphaA_average_air = sum(moistAirCell.heatTransfer.alphaA),
              mass_component = sum(moistAirCell.geometricWallMass),
              geometry = hxGeometry.summary) if includeDefaultSummary
            annotation (Placement(transformation(extent={{80,100},{100,120}}, rotation=0)));

          protected
            parameter String hxType="FinAndTube" "HX type";

          initial equation
            if (initVLEFluid == "constantEnthalpy") then
              vleFluidCell.vleFluid.h = hInitialVLEFluid*ones(nCells);
            elseif (initVLEFluid == "linearEnthalpyDistribution") then
              if nCells > 1 then
                vleFluidCell.vleFluid.h = linspace(hInitialVLEFluid_Cell1,hInitialVLEFluid_CellN,nCells);
              else
                terminate("Initialization with linear enthalpy distrubution requires discretization >1!");
              end if;
            elseif (initVLEFluid == "constantTemperature") then
              vleFluidCell.vleFluid.h = TILMedia.VLEFluidFunctions.specificEnthalpy_pTxi(vleFluidType, vleFluidCell.vleFluid.p,TInitialVLEFluid,mixingRatioInitialVLEFluid[1:end-1]/sum(mixingRatioInitialVLEFluid));
            elseif (initVLEFluid == "steadyState") then
              der(vleFluidCell.vleFluid.h) = zeros(nCells);
            elseif (initVLEFluid == "userSpecifiedValues") then
              vleFluidCell.vleFluid.h = hInitialUserValues;
            else
              terminate("Initialization method not yet implemented in Fin-and-tube heat exchanger!");
            end if;

            for i in 1 : nCells loop
              vleFluidCell[i].vleFluid.xi = mixingRatioInitialVLEFluid[1:end-1]/sum(mixingRatioInitialVLEFluid);
            end for;

            assert(gasType.condensingIndex>0,"no condensing component");
            assert(PI*hxGeometry.tubeOuterDiameter*hxGeometry.tubeOuterDiameter/4 <
                  hxGeometry.serialTubeDistance*hxGeometry.parallelTubeDistance, "Distance from tube to tube is smaller than outer diameter of the tube;
         Reduce tubeInnerDiameter or enlarge serialTubeDistance and parallelTubeDistance to solve the problem");
            assert(hxGeometry.geometryIsValid, "Geometry is not valid");
            assert(sim.dpdtPort[pressureStateID].counter>0.5,"Pressure state ID is not valid");

          equation
            connect(sim.fluidPort[vleFluidType.ID],simPort.vleFluidPort);
            simPort.vleFluidMass
                             =cumulatedVLEFluidMass;
            simPort.vleFluidVolume
                               =cumulatedVLEFluidVolume;

            annotation (
              Icon(coordinateSystem(
                  preserveAspectRatio=false,
                  extent={{-140,-140},{140,140}},
                  initialScale=0.1), graphics),
              Diagram(coordinateSystem(
                  preserveAspectRatio=false,
                  extent={{-140,-140},{140,140}},
                  initialScale=0.1), graphics));
          end PartialHX;
        end BaseClasses;
        annotation(classOrder={"ParallelFlowHX","CrossFlowHX","PortArrayCrossFlowHX","*","BaseClasses","Testers"});
      end MoistAirVLEFluid;

      package Geometry
       extends TIL.Internals.ClassTypes.ModelPackage;

        record FinAndTubeGeometry "Geometry of fin and tube heat exchanger"
          extends TIL.Internals.ClassTypes.Record;

        /* ************************************************************************************
* Input parameters                                                                    *
* *********************************************************************************** */
          parameter SI.Length finnedTubeLength(min=Modelica.Constants.eps)
            "Length of finned tubes"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter Integer nSerialTubes(min=1) "Number of serial tubes"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter SI.Distance serialTubeDistance(min=Modelica.Constants.eps)
            "Distance between serial tubes"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter Integer nParallelTubes(min=1) "Number of parallel tubes"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter SI.Distance parallelTubeDistance(min=Modelica.Constants.eps)
            "Distance between parallel tubes"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter SI.Thickness finThickness(min=Modelica.Constants.eps)
            "Thickness of fins"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter SI.Distance finPitch(min=Modelica.Constants.eps)
            "Distance between fins"
            annotation(Dialog(group="Fin Side Geometry"));
          parameter SI.Diameter tubeInnerDiameter(min=Modelica.Constants.eps)
            "Inner diameter of tube"
            annotation (Dialog(group="Tube Side Geometry"));
          parameter SI.Thickness tubeWallThickness(min=Modelica.Constants.eps)
            "Thickness of tube walls"
            annotation (Dialog(group="Tube Side Geometry"));
          parameter Integer nTubeSideParallelHydraulicFlows(min=1)=1
            "Number of parallel tube side flows"
            annotation (Dialog(group="Tube Side Geometry"));

        /* ************************************************************************************
* Calculated parameters for entire heat exchanger                                     *
* *********************************************************************************** */
          final parameter Boolean geometryIsValid = totalFinnedVolume>0 and totalFinSideHeatTransferArea>0
                          and totalTubeSideHeatTransferArea>0 and totalTubeInnerVolume>0
                          and totalTubeOuterVolume>0 and totalFinVolume>0;

          final parameter SI.Length totalHXHeight=nParallelTubes*parallelTubeDistance
            "Total height of the heat exchanger";
          final parameter SI.Length totalHXWidth=finnedTubeLength
            "Total width of the heat exchanger";
          final parameter SI.Length totalHXDepth= nSerialTubes*serialTubeDistance
            "Total depth of the heat exchanger";
          final parameter Integer totalNTubes=nParallelTubes*nSerialTubes
            "Total number of tubes";
          final parameter SI.Diameter tubeOuterDiameter=tubeInnerDiameter+2.0*tubeWallThickness
            "Outer diameter of tube";
          final parameter Integer nFins = integer(finnedTubeLength/finPitch) + 1
            "Total number of fins";
          final parameter SI.Length totalFinnedTubeLength=totalNTubes*finnedTubeLength
            "Total finned length of all tubes";
          final parameter SI.Volume totalFinnedVolume=totalHXHeight*totalHXWidth*totalHXDepth
            "Total finned volume";
          final parameter SI.Area totalFinSideHeatTransferArea=(singleTubeFinSurfaceArea + singleTubeOuterSurfaceArea)*totalNTubes
            "Total fin side heat transfer area";
          final parameter SI.Area totalTubeSideHeatTransferArea=singleTubeInnerSurfaceArea*totalNTubes
            "Total tube side heat transfer area";
          final parameter SI.Volume totalTubeInnerVolume=singleTubeInnerVolume*totalNTubes
            "Total tube inner volume";
          final parameter SI.Volume totalTubeOuterVolume=singleTubeOuterVolume*totalNTubes
            "Total tube outer volume";
          final parameter SI.Volume totalFinVolume=(totalHXHeight*totalHXDepth*finThickness - PI*tubeOuterDiameter*tubeOuterDiameter/4*finThickness*totalNTubes)*nFins
            "Total fin volume";
          final parameter SI.Area totalFinSideCrossSectionalArea = totalHXHeight*totalHXWidth
            "Total air side cross sectional area";
          final parameter Real finSideHeatTransferAreaRatio=singleTubeFinSurfaceArea/(singleTubeFinSurfaceArea + singleTubeOuterSurfaceArea)
            "Fraction of fins on fin side heat transfer area";
          final parameter Real voidRatio = 1 - finThickness/finPitch - (PI*tubeOuterDiameter*tubeOuterDiameter*(finPitch - finThickness))/(4.0*serialTubeDistance*parallelTubeDistance*finPitch)
            "Void ratio";
          final parameter Real areaRatio = 1 - tubeOuterDiameter/parallelTubeDistance - (finThickness*(parallelTubeDistance - tubeOuterDiameter))/(parallelTubeDistance*finPitch)
            "Area ratio";

        /* ************************************************************************************
* Calculated parameters for single tube side path                                   *
* *********************************************************************************** */
          final parameter SI.Length tubeSidePathLength=totalFinnedTubeLength/nTubeSideParallelHydraulicFlows
            "Length of a single tube side path";
          final parameter SI.Area tubeSidePathInnerSurfaceArea=PI*tubeInnerDiameter*tubeSidePathLength
            "Tube side path tube inner surface area";
          final parameter SI.Volume tubeSidePathInnerVolume=PI*tubeInnerDiameter*tubeInnerDiameter/4.0*tubeSidePathLength
            "Tube side path tube inner volume";
          final parameter SI.Area tubeSidePathHeatTransferArea=tubeSidePathInnerSurfaceArea
            "Tube side path heat transfer area";

        /* ************************************************************************************
* Calculated parameters for single tube                                               *
* *********************************************************************************** */
          final parameter SI.Area singleTubeFinSurfaceArea=2*(serialTubeDistance*parallelTubeDistance - PI*tubeOuterDiameter*tubeOuterDiameter/4)*nFins
            "Fin surface area of a single tube";
          final parameter SI.Area singleTubeInnerSurfaceArea = PI*tubeInnerDiameter*finnedTubeLength
            "Inner surface area of single tube (for finned length)";
          final parameter SI.Area singleTubeOuterSurfaceArea=PI*tubeOuterDiameter*finnedTubeLength - PI*tubeOuterDiameter*finThickness*nFins
            "Outer surface area of single tube (for finned length)";
          final parameter SI.Volume singleTubeInnerVolume = PI*tubeInnerDiameter*tubeInnerDiameter/4.0*finnedTubeLength
            "Inner volume of single tube (for finned length)";
          final parameter SI.Volume singleTubeOuterVolume = PI*tubeOuterDiameter*tubeOuterDiameter/4.0*finnedTubeLength
            "Outer volume of single tube (for finned length)";

        /* ************************************************************************************
* Summary                                                                             *
* *********************************************************************************** */

          Summary summary(
              final finnedTubeLength=finnedTubeLength,
              final nSerialTubes = nSerialTubes,
              final serialTubeDistance=serialTubeDistance,
              final nParallelTubes=nParallelTubes,
              final parallelTubeDistance=parallelTubeDistance,
              final finThickness=finThickness,
              final finPitch=finPitch,
              final tubeInnerDiameter=tubeInnerDiameter,
              final tubeWallThickness=tubeWallThickness,
              final nTubeSideParallelHydraulicFlows=nTubeSideParallelHydraulicFlows,
              final tubeSidePathLength=tubeSidePathLength,
              final tubeOuterDiameter=tubeOuterDiameter,
              final totalNTubes=totalNTubes,
              final totalFinnedVolume=totalFinnedVolume,
              final totalFinSideHeatTransferArea=totalFinSideHeatTransferArea,
              final totalTubeSideHeatTransferArea=totalTubeSideHeatTransferArea,
              final totalTubeInnerVolume=totalTubeInnerVolume,
              final totalHXHeight=totalHXHeight,
              final totalHXWidth=totalHXWidth,
              final totalHXDepth=totalHXDepth,
              final totalFinSideCrossSectionalArea=totalFinSideCrossSectionalArea);

          annotation(Documentation(info="<html>
<p>This record contains all the geometric information required to describe a fin-and-tube heat exchanger. The most important variables can be explained with the following schematic drawing of the heat exchanger (neglecting the fins for now):</p>
<img src=\"../Images/BasicFinAndTube.png\">
<p>The length of finned tubes for this example is assumed to be the length of one of the 27 tubes, the number of serial tubes is 3 for this example and the number of parallel tubes is 9. The shown heat exchanger has 3 parallel vleFluid flow paths.</p>
<img src=\"../Images/BasicFinAndTube2.png\">
</html>"),  Images(
              Dialog(
              tab="General",
              group="Fin Side Geometry",
              source="./Images/BasicFinAndTube_small.png"),
              Dialog(
              tab="General",
              group="Tube Side Geometry",
              source="./Images/BasicFinAndTube3_small.png")));
        end FinAndTubeGeometry;

        record KKI001 "KKI001"
          extends TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry(
            finnedTubeLength=0.6,
            nSerialTubes=5,
            serialTubeDistance=22e-3,
            nParallelTubes=8,
            parallelTubeDistance=25.4e-3,
            finThickness=0.2e-3,
            finPitch=2.2e-3,
            tubeInnerDiameter=7e-3,
            tubeWallThickness=1.5e-3,
            nTubeSideParallelHydraulicFlows=4);
        end KKI001;

        record Summary
           extends TIL.Internals.ClassTypes.Record;
        parameter SI.Length finnedTubeLength(min=Modelica.Constants.eps)
            "Length of finned tubes";
        parameter Integer nSerialTubes(min=1) "Number of serial tubes";
        parameter SI.Distance serialTubeDistance(min=Modelica.Constants.eps)
            "Distance between serial tubes";
        parameter Integer nParallelTubes(min=1) "Number of parallel tubes";
        parameter SI.Distance parallelTubeDistance(min=Modelica.Constants.eps)
            "Distance between parallel tubes";
        parameter SI.Thickness finThickness(min=Modelica.Constants.eps)
            "Thickness of fins";
        parameter SI.Distance finPitch(min=Modelica.Constants.eps)
            "Distance between fins";
        parameter SI.Diameter tubeInnerDiameter(min=Modelica.Constants.eps)
            "Inner diameter of tube";
        parameter SI.Thickness tubeWallThickness(min=Modelica.Constants.eps)
            "Thickness of tube walls";
        parameter Integer nTubeSideParallelHydraulicFlows(min=1)=1
            "Number of parallel tube side flows";
        parameter SI.Length tubeSidePathLength "Length of a single tube side path";
        parameter SI.Diameter tubeOuterDiameter "Outer diameter of tube";
        parameter Integer totalNTubes "Total number of tubes";
        parameter SI.Volume totalFinnedVolume "Total finned volume";
        parameter SI.Area totalFinSideHeatTransferArea
            "Total fin side heat transfer area";
        parameter SI.Area totalTubeSideHeatTransferArea
            "Total tube side heat transfer area";
        parameter SI.Volume totalTubeInnerVolume "Total tube inner volume";
        parameter SI.Length totalHXHeight "Total height of the heat exchanger";
        parameter SI.Length totalHXWidth "Total width of the heat exchanger";
        parameter SI.Length totalHXDepth "Total depth of the heat exchanger";
        parameter SI.Area totalFinSideCrossSectionalArea
            "Total air side cross sectional area";
        end Summary;
        annotation(classOrder={"FinAndTubeGeometry","*","Testers"});
      end Geometry;

      package TransportPhenomena
      extends TIL.Internals.ClassTypes.ModelPackage;

        package FinEfficiency
          extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialFinEfficiency
             extends TIL.Cells.TransportPhenomena.PartialFinEfficiency;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";
          end PartialFinEfficiency;

          model ConstFinEfficiency "Constant fin efficiency"
           extends PartialFinEfficiency;

            parameter SI.Efficiency constValueFinEfficiency(max=1.0, min=0.0) = 1.0;

          equation
            eta = constValueFinEfficiency;
          end ConstFinEfficiency;

          model Schmidt "1-D Approximation (Schmidt)"
            extends PartialFinEfficiency;

            final parameter Real diameterRatio=
                1.27*hxGeometry.parallelTubeDistance/hxGeometry.tubeOuterDiameter * ((0.25+(hxGeometry.serialTubeDistance/hxGeometry.parallelTubeDistance)^2)^0.5 - 0.3)^0.5
              "Dimensionless diameter ratio";
            final parameter SI.Length effectiveFinHeight=
                0.5*hxGeometry.tubeOuterDiameter*(diameterRatio-1)*(1+0.35*Modelica.Math.log(diameterRatio))
              "Effective fin height";

          protected
            Real X( start=1) "Auxiliary variable";

          equation
            X = effectiveFinHeight*(2*alpha/finMaterial.lambda/hxGeometry.finThickness)^0.5;
            eta = Modelica.Math.tanh(X)/X;
          end Schmidt;
          annotation(classOrder={"PartialFinEfficiency","*","Testers","Internals"});
        end FinEfficiency;

        package FinSideHeatTransfer "Fin side heat transfer models"
         extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialHeatTransfer
            "Base class for fin side heat transfer of fin and tube HXs"

            extends TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer(
              final useAlphaAState = false);

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

              /*

The objective is the calculation of a heat transfer value.
The result is returned to the variable:

  -> alphaA
  
A typical heat transfer model needs input variables for its calculation.
 
This base class provides the following information from
TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer using the inner outer concept
 
  - mdotHydraulic                                  - mass flow rate per typical hydraulic cross section
  - QdotHydraulic                                  - heat flow rate per hydraulic flow
  - wallTemperature                                - temperature of wall
  - properties                                     - properties of fin side medium, accessible by writing e.g. "properties.d"
    .d, .h, .p, .T, .cp, .s   
    .transp.Pr, .lambda, .eta, .sigma               
  - cellGeometry                                   - geometry corresponding to cell model
    .length
    .volume
    .heatTransferArea
    .hydraulicCrossSectionalArea
    .finHeatTransferAreaRatio
    .nParallelHydraulicFlows
 
plus information from heat exchanger model:
 
  - hxGeometry                                   - geometry corresponding to heat exchanger model
      .finnedTubeLength
      .nSerialTubes
      .serialTubeDistance
      .nParallelTubes
      .parallelTubeDistance
      .finThickness
      .finPitch
      .tubeInnerDiameter
      .tubeWallThickness
      .nTubeSideParallelHydraulicFlows
      .tubeSidePathLength
      .tubeOuterDiameter
      .totalNTubes
      .totalFinnedVolume
      .totalFinSideHeatTransferArea
      .totalTubeSideHeatTransferArea
      .totalTubeInnerVolume
      .totalHXHeight
      .totalHXWidth
      .totalHXDepth
      .totalFinSideCrossSectionalArea
  */

          end PartialHeatTransfer;

          model ConstantAlpha "Constant alpha"
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.PartialHeatTransfer(
                final computeTransportProperties = false);
            parameter SI.CoefficientOfHeatTransfer constantAlpha
              "Constant value for alpha";

          equation
            alphaA = constantAlpha*cellGeometry.heatTransferArea;
          end ConstantAlpha;
          annotation(classOrder={"PartialHeatTransfer","ConstantAlpha","ConstantAlphaA","MassFlowDependentAlphaA","Haaf","FinSideHeatTransferModels","*","Testers","Internals"});
        end FinSideHeatTransfer;

        package FinSidePressureDrop "Fin side pressure drop models"
         extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialPressureDrop
            "Base class for fin side pressure drop of fin and tube HXs"

             extends TIL.Cells.TransportPhenomena.PartialPressureDrop;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

              /*

The objective is the calculation of a pressure drop value.
The result is returned to the variable:

  -> pressureDrop
  
A typical pressure drop model needs input variables for its calculation.
 
This base class provides the following information from
TIL.Cells.TransportPhenomena.PartialPressureDrop using the inner outer concept
 
  - mdotHydraulic                                  - mass flow rate per typical hydraulic cross section  
  - properties                                     - properties of fin side medium, accessible by writing e.g. "properties.d"
    .d, .h, .p, .T, .cp, .s
    .transp.Pr, .lambda, .eta, .sigma                           
  - cellGeometry                                   - geometry corresponding to cell model
    .length
    .volume
    .heatTransferArea
    .hydraulicCrossSectionalArea
    .finHeatTransferAreaRatio
    .nParallelHydraulicFlows
 
plus information from heat exchanger model:
 
  - hxGeometry                                   - geometry corresponding to heat exchanger model
    .finnedTubeLength
    .nSerialTubes
    .serialTubeDistanc
    .nParallelTubes
    .parallelTubeDistance
    .finThickness
    .finPitch
    .tubeInnerDiameter
    .tubeWallThickness
    .nTubeSideParallelHydraulicFlows
    .tubeSidePathLength
    .tubeOuterDiameter
    .totalNTubes
    .totalFinnedVolume
    .totalFinSideHeatTransferArea
    .totalTubeSideHeatTransferArea
    .totalTubeInnerVolume
    .totalHXHeight
    .totalHXWidth
    .totalHXDepth
    .totalFinSideCrossSectionalArea
  */

          end PartialPressureDrop;

          model ZeroPressureDrop "pressure drop = 0 Pa"
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSidePressureDrop.PartialPressureDrop(
                final computeTransportProperties = false);

          equation
            pressureDrop = 0.0;
          end ZeroPressureDrop;
          annotation(classOrder={"PartialPressureDrop","*","Testers"});
        end FinSidePressureDrop;

        package TubeSideHeatTransfer "Tube side heat transfer models"
         extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialLiquidHeatTransfer
            "Base class for tube side heat transfer of fin and tube HXs, valid for liquids"

            extends TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

              /*

The objective is the calculation of a heat transfer value.
The result is returned to the variable:

  -> alphaA
  
A typical heat transfer model needs input variables for its calculation.
 
This base class provides the following information from
TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer using the inner outer concept
 
  - mdotHydraulic                                  - mass flow rate per typical hydraulic cross section
  - QdotHydraulic                                  - heat flow rate per hydraulic flow
  - wallTemperature                                - temperature of wall
  - properties                                     - properties of tube side medium, accessible by writing e.g. "properties.d"
    .d, .h, .p, .T, .cp, .s   
    .transp.Pr, .lambda, .eta, .sigma               
  - cellGeometry                                   - geometry corresponding to cell model
    .length
    .volume
    .heatTransferArea
    .hydraulicCrossSectionalArea
    .finHeatTransferAreaRatio
    .nParallelHydraulicFlows
 
plus information from heat exchanger model:
 
  - hxGeometry                                   - geometry corresponding to heat exchanger model
      .finnedTubeLength
      .nSerialTubes
      .serialTubeDistance
      .nParallelTubes
      .parallelTubeDistance
      .finThickness
      .finPitch
      .tubeInnerDiameter
      .tubeWallThickness
      .nTubeSideParallelHydraulicFlows
      .tubeSidePathLength
      .tubeOuterDiameter
      .totalNTubes
      .totalFinnedVolume
      .totalFinSideHeatTransferArea
      .totalTubeSideHeatTransferArea
      .totalTubeInnerVolume
      .totalHXHeight
      .totalHXWidth
      .totalHXDepth
      .totalFinSideCrossSectionalArea
  */

          end PartialLiquidHeatTransfer;

          partial model PartialVLEFluidHeatTransfer
            "Base class for tube side heat transfer of fin and tube HXs, valid for vleFluids"

            extends TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

              /*

The objective is the calculation of a heat transfer value.
The result is returned to the variable:

  -> alphaA
  
A typical heat transfer model needs input variables for its calculation.
 
This base class provides the following information from
TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer using the inner outer concept
 
  - mdotHydraulic                                  - mass flow rate per typical hydraulic cross section
  - QdotHydraulic                                  - heat flow rate per hydraulic flow
  - wallTemperature                                - temperature of wall
  - properties                                     - properties of tube side medium, accessible by writing e.g. "properties.d"
    .d, .h, .p, .T, .cp, .s, .q                    
    .crit.d, .T, .p, .h, .s                            
    .VLE.d_l, .h_l, .p_l, .s_l, .T_l,
        .d_v, .h_v, .p_v, .s_v, .T_v
    .VLETransp.Pr_l, .Pr_v, .eta_l, .eta_v,
        .lambda_l, .lambda_v     
    .transp.Pr, .lambda, .eta, .sigma               
  - cellGeometry                                   - geometry corresponding to cell model
    .length
    .volume
    .heatTransferArea
    .hydraulicCrossSectionalArea
    .finHeatTransferAreaRatio
    .nParallelHydraulicFlows
 
plus information from heat exchanger model:
 
  - hxGeometry                                   - geometry corresponding to heat exchanger model
      .finnedTubeLength
      .nSerialTubes
      .serialTubeDistance
      .nParallelTubes
      .parallelTubeDistance
      .finThickness
      .finPitch
      .tubeInnerDiameter
      .tubeWallThickness
      .nTubeSideParallelHydraulicFlows
      .tubeSidePathLength
      .tubeOuterDiameter
      .totalNTubes
      .totalFinnedVolume
      .totalFinSideHeatTransferArea
      .totalTubeSideHeatTransferArea
      .totalTubeInnerVolume
      .totalHXHeight
      .totalHXWidth
      .totalHXDepth
      .totalFinSideCrossSectionalArea
  */

          end PartialVLEFluidHeatTransfer;

          model ConstantAlpha "Constant alpha"
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.PartialLiquidHeatTransfer(
                       final computeTransportProperties = false, final useAlphaAState = false);
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.PartialVLEFluidHeatTransfer(
                    final computeTransportProperties = false, final useAlphaAState = false);

            parameter SI.CoefficientOfHeatTransfer constantAlpha
              "Constant value for alpha";

          equation
            alphaA = constantAlpha*cellGeometry.heatTransferArea;
          end ConstantAlpha;
          annotation(classOrder={"PartialLiquidHeatTransfer","PartialVLEFluidHeatTransfer","ConstantAlpha","ConstantAlphaA","Polifke","LinearGnielinskiDittusBoelter","*","Testers"});
        end TubeSideHeatTransfer;

        package TubeSidePressureDrop "Tube side pressure drop models"
         extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialLiquidPressureDrop
            "Base class for liquid tube side pressure drop of fin and tube HXs"

            extends TIL.Cells.TransportPhenomena.PartialPressureDrop;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

              /*

The objective is the calculation of a pressure drop value.
The result is returned to the variable:

  -> pressureDrop
  
A typical pressure drop model needs input variables for its calculation.
 
This base class provides the following information from
TIL.Cells.TransportPhenomena.PartialPressureDrop using the inner outer concept
 
  - mdotHydraulic                                  - mass flow rate per typical hydraulic cross section  
  - properties                                     - properties of tube side medium, accessible by writing e.g. "properties.d"
    .d, .h, .p, .T, .cp, .s
    .transp.Pr, .lambda, .eta, .sigma                         
  - cellGeometry                                   - geometry corresponding to cell model
    .length
    .volume
    .heatTransferArea
    .hydraulicCrossSectionalArea
    .finHeatTransferAreaRatio
    .nParallelHydraulicFlows
 
plus information from heat exchanger model:
 
  - hxGeometry                                   - geometry corresponding to heat exchanger model
    .finnedTubeLength
    .nSerialTubes
    .serialTubeDistanc
    .nParallelTubes
    .parallelTubeDistance
    .finThickness
    .finPitch
    .tubeInnerDiameter
    .tubeWallThickness
    .nTubeSideParallelHydraulicFlows
    .tubeSidePathLength
    .tubeOuterDiameter
    .totalNTubes
    .totalFinnedVolume
    .totalFinSideHeatTransferArea
    .totalTubeSideHeatTransferArea
    .totalTubeInnerVolume
    .totalHXHeight
    .totalHXWidth
    .totalHXDepth
    .totalFinSideCrossSectionalArea
  */

          end PartialLiquidPressureDrop;

          partial model PartialVLEFluidPressureDrop
            "Base class for vleFluid tube side pressure drop of fin and tube HXs"

            extends TIL.Cells.TransportPhenomena.PartialPressureDrop;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

              /*

The objective is the calculation of a pressure drop value.
The result is returned to the variable:

  -> pressureDrop
  
A typical pressure drop model needs input variables for its calculation.
 
This base class provides the following information from
TIL.Cells.TransportPhenomena.PartialPressureDrop using the inner outer concept
 
  - mdotHydraulic                                  - mass flow rate per typical hydraulic cross section    
  - properties                                     - properties of tube side medium, accessible by writing e.g. "properties.d"
    .d, .h, .p, .T, .cp, .s, .q 
    .crit.d, .T, .h, .s                            
    .sat.pl, .pv, .Tl, .Tv, .dl, .dv, .hl, .hv     
    .transp.Pr, .lambda, .eta, .sigma
  - cellGeometry                                   - geometry corresponding to cell model
    .length
    .volume
    .heatTransferArea
    .hydraulicCrossSectionalArea
    .finHeatTransferAreaRatio
    .nParallelHydraulicFlows
 
plus information from heat exchanger model:
 
  - hxGeometry                                   - geometry corresponding to heat exchanger model
    .finnedTubeLength
    .nSerialTubes
    .serialTubeDistanc
    .nParallelTubes
    .parallelTubeDistance
    .finThickness
    .finPitch
    .tubeInnerDiameter
    .tubeWallThickness
    .nTubeSideParallelHydraulicFlows
    .tubeSidePathLength
    .tubeOuterDiameter
    .totalNTubes
    .totalFinnedVolume
    .totalFinSideHeatTransferArea
    .totalTubeSideHeatTransferArea
    .totalTubeInnerVolume
    .totalHXHeight
    .totalHXWidth
    .totalHXDepth
    .totalFinSideCrossSectionalArea
  */
          end PartialVLEFluidPressureDrop;

          model ZeroPressureDrop "pressure drop = 0 Pa"
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.PartialLiquidPressureDrop(
                      final computeTransportProperties = false);
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.PartialVLEFluidPressureDrop(
                      final computeTransportProperties = false);

          equation
            pressureDrop = 0.0;
          end ZeroPressureDrop;
          annotation(classOrder={"PartialLiquidPressureDrop","PartialVLEFluidPressureDrop","*","Testers"});
        end TubeSidePressureDrop;

        package WallHeatTransfer "Wall cell heat transfer"
         extends TIL.Internals.ClassTypes.ModelPackage;

          partial model PartialHeatTransfer
           extends TIL.Cells.TransportPhenomena.PartialSolidHeatTransfer;

          protected
            outer parameter TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry
              hxGeometry "HX geometry record";

          end PartialHeatTransfer;

          model ConstantR
            "Constant heat resistance of the wall (independent of geometry)"
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.PartialHeatTransfer;
            parameter SI.ThermalResistance constantR
              "Constant heat resistance of the wall (independent of geometry)";

          equation
          R_NS = constantR*(hxGeometry.totalTubeOuterVolume - hxGeometry.totalTubeInnerVolume)/cellGeometry.volume;
          R_WE = 1e12;

          end ConstantR;

          model GeometryBasedConduction
            extends
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.PartialHeatTransfer;

          protected
            SI.ThermalResistance R_NS_total "Thermal resistance (N-S) for total HX";

          equation
            R_NS_total = Modelica.Math.log(hxGeometry.tubeOuterDiameter/hxGeometry.tubeInnerDiameter)/(2*PI*properties.lambda*hxGeometry.totalFinnedTubeLength);
            R_NS = R_NS_total*(hxGeometry.totalTubeOuterVolume - hxGeometry.totalTubeInnerVolume)/cellGeometry.volume;
            R_WE = 1e12;
          end GeometryBasedConduction;
          annotation(classOrder={"PartialHeatTransfer","*","Testers"});
        end WallHeatTransfer;
      end TransportPhenomena;
      annotation(classOrder={"*","Geometry","TransportPhenomena"});
    end FinAndTube;

    package SplitterJoiner
     extends TIL.Internals.ClassTypes.ModelPackage;

      model GasSplitterJoiner

        parameter TILMedia.GasTypes.BaseGas gasType "Gas type";

      /******************** Connector *****************************/
        TIL.Connectors.GasPort inlet(final gasType=gasType) "Inlet fluid port"
          annotation (Placement(transformation(extent={{-270,-10},{-250,10}},
                                                                            rotation=0),
              iconTransformation(extent={{-270,-10},{-250,10}})));
        TIL.Connectors.GasPort[nPorts1, nPorts2] outlets(each final gasType=gasType)
          "Outlet fluid ports"
          annotation (Placement(transformation(extent={{-150,-10},{-130,10}},
                                                                          rotation=0),
              iconTransformation(extent={{-150,-10},{-130,10}})));

        TIL.Connectors.GasPort outlet( final gasType=gasType) "Outlet fluid port"
          annotation (Placement(transformation(extent={{250,-10},{270,10}},
                                                                          rotation=0),
              iconTransformation(extent={{250,-10},{270,10}})));
        TIL.Connectors.GasPort[nPorts1, nPorts2] inlets(each final gasType=gasType)
          "Inlet fluid ports"
          annotation (Placement(transformation(extent={{130,-10},{150,10}}, rotation=
                  0), iconTransformation(extent={{130,-10},{150,10}})));

        parameter Integer nPorts1(min=1) "Number of ports in first dimension";
        parameter Integer nPorts2(min=1) "Number of ports in second dimension";

        parameter Real[nPorts1, nPorts2] weightingFactor = ones(nPorts1, nPorts2)
          "Ratio of output quantity" annotation(Dialog(tab="Advanced"));

        parameter TIL.Internals.CellFlowType cellFlowType = "flow A-B";
      protected
        final parameter Real totalWeightingFactor = sum(weightingFactor);

      equation
      /***************** Splitter **************************/

        outlets.p = inlet.p*ones(nPorts1, nPorts2);

        if (cellFlowType=="flow A-B") then
          0 = inlet.h_outflow;
          zeros(gasType.nc-1) = inlet.xi_outflow;
        else
          0 = inlet.h_outflow*inlet.m_flow + sum(outlets[i,j].m_flow*inStream(outlets[i,j].h_outflow) for i in 1:nPorts1, j in 1:nPorts2);
          zeros(gasType.nc-1) = inlet.xi_outflow*inlet.m_flow + sum(outlets[i,j].m_flow*inStream(outlets[i,j].xi_outflow) for i in 1:nPorts1, j in 1:nPorts2);
        end if;

        outlets.h_outflow = ones(nPorts1, nPorts2)*inStream(inlet.h_outflow)
          "Energy balance";

        outlets.xi_outflow = fill(inStream(inlet.xi_outflow), nPorts1, nPorts2);
        outlets.m_flow = -inlet.m_flow*weightingFactor/totalWeightingFactor
          "Splitting of mass flow, fullfilling mass conservation";

      /***************** Joiner **************************/

        outlet.p = sum(inlets.p)/(nPorts1*nPorts2);

        0.0 = outlet.h_outflow*outlet.m_flow + sum(inlets[i,j].m_flow*inStream(inlets[i,j].h_outflow) for i in 1:nPorts1, j in 1:nPorts2)
          "Energy balance";
        zeros(gasType.nc-1) = outlet.xi_outflow*outlet.m_flow + sum(inlets[i,j].m_flow*inStream(inlets[i,j].xi_outflow) for i in 1:nPorts1, j in 1:nPorts2)
          "Mass balance";

        if (cellFlowType=="flow A-B") then
          inlets.h_outflow = fill(0, nPorts1, nPorts2);
          inlets.xi_outflow = fill(zeros(gasType.nc-1), nPorts1, nPorts2);
        else
          inlets.h_outflow = fill(inStream(outlet.h_outflow), nPorts1, nPorts2);
          inlets.xi_outflow = fill(inStream(outlet.xi_outflow), nPorts1, nPorts2);
        end if;

        outlet.m_flow + sum(inlets.m_flow) = 0.0 "Mass balance";

        annotation (Diagram(coordinateSystem(extent={{-260,-100},{260,100}},
                preserveAspectRatio=false),
                            graphics),
                             Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-260,
                  -100},{260,100}}),
                                  graphics={                                 Bitmap(
                  extent={{140,-100},{260,100}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAHgAAADICAIAAACszLLwAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA3hJREFUeNrs3dFt4kAUheGwDeASoALowFAB7gBKoANKgA6gBHcAJZgKcAdABd6RIlmjbNAKgc29Z/7zRKIoxJ8mcxzHM/76IoQQYjWD+IOmafp748HgI+/bK250jH8Ya/0EaKCBJkADDTQEQANNgAYaaAI00ARooIEmQANNgAYaaAI00ARooIEmQANNgAYaaAI00ARooIEmQANNgAYaaAI00ARooIEmQAP9SobDYfu6qiqgu8p0Om1fn04noLtKURTt68PhAHQf0OfzWX5Qfwx6NBrled5+uF6vb7dbKs3Y9Jvj8Ri/+3K5bLRiBTokHtQh2+1WFfpj+959J0wXYQ653+/tZ8Iwn81mGjOEoX3vsiwry/JHSdZ1zRzdScKMEf8Yk8nker0yR3eS0IR6xWgROgzhMJDFitEidMjlcokvgHwXI9B9nFkH96APNMXoGVqpGK1DyxSjdWiZYnQArVGMT0Dv9/sfI+vFf1+Fb5hOMT4B/Ubl1jqdYvQE7boY3Uwd3ovRRxkKFKM/aKfF6BLaYzF6hXZXjF6h3RWjY2hfxegb2lExuof2UowK0C6KUQHaRTGKQNsvRh1o48UoBW25GNWgzRajGrTZYhSEtlmMmtAGi1EW2loxKkObKkZlaFPFKA5tpxj1of8txjzPge6pGPsf1M7u63ilGOMf/lErvn6Mj47L051KL2axWLRvvdlsfv2atxzjr8dlYi14avkP9G63e+/UEb5hn8ty400Tsizr6BifPi7KkLMO96d3H15031HCjDGfz+Nf7bquH00dSSy67yLBNN7eJqQsy/6VxedoLipxmZQL/0B7vGInDs0/Z5MrQGVobqBJsQA1obnJMd0CVIPmRvTUC1AKmsVCFKAKNAs6KUAhaBbdU4Aq0Pobo7DVT0/QbF6VCnQq27GxwaB+Gaa1ZSZ/ASpDs60xfwEKQbP1PAWoAs3jQShAIWge4UQBqkCLFeAjaB4cmcYalqIoYuUwh8goG1rDwsN9e0r8ZF+ZAjQ3R9d1PR6PJadmW3N0/EzfMJzlp2YT0KvVSr7/PgZdVVX7OoUzjY/N0fH81aisQbd7Hp1OgAYaaAI00EBDADTQBGiggSZAA02ABhpoAjTQBGiggSZAA02ABhpoAjTQBGiggSZAA02ABhpoAjTQBGiggSZAA02ABpoQQggxl78CDABr6TDzUkAVAwAAAABJRU5ErkJggg==",
                fileName="modelica://TIL/Images/SplitterJoiner_right.png"), Bitmap(
                extent={{-260,-100},{-140,100}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAHgAAADICAIAAACszLLwAAAACXBIWXMAAAsTAAALEwEAmpwYAAADpklEQVR42u3d7XGjMBSF4XgbkDuwO3AJ4ApMB6QEd+ASoANcAh1ACbgC6AC5AuUfo3Xi2cxii6vLe37Zk50JepbRCeLr44MQQojUbPwvzjmdg9xsFhmj/3v/sK+FCdBAA02ABhpoCIAGmgANNNAEaKAJ0EADTYAGmgANNNAEaKAJ0EADTYAGmgANNNAEaKAJ0EADTYAGmgANNNAEaKAJ0EADTYAGOop0XTd93u12QL8rbdtOn/f7PdDvyvV6nT5nWQb0u3bn2+0G9HtjrT2fz9PXJEkWnDr+itOVPM/90TVNE3gDVgFdFIU/tCRJwm+DvwE6n3vXtu3xeJy+GmOGYdhut4E3Q/lz74ZheCi9uq7DKyufo8dxPBwO/qCKolhqYzRDPxRgnucLboxa6IcCPBwO4zgC/eI0TeOPxRjT9/2ym6QQuu97Y8yyfzXrhxZVgJqhRRWgWmhpBagTWmABKoSWWYDaoGcWYFVVD/9Jc2KMqapKJ/TMAnyh8mStEHp+AQIdqACZOqIvQA3QYo8AtUGLPQLUAz2Oo+QjwN9AR3DO0Frbdd3DOcCu66RcO6DmnKG19vs5QPnKkZ0zjK4AY52joyvAKKFjLMD4oIUvgSqBjusIMFboqAswJuioCzAa6NgLMA5oBQUYAbSOApQOraYApUOrKUDR0JoKUC60sgJ8Br3werS1dr/f3+93vwDTNNVxk4eg9egsy3zloijUKAtaj+77Xl8B/uccPf/6h2fXPDx0oDEmZAGKu67jJVvz41U8zrnL5TL9m9PpFHJfC3+lEg9GCZR/QJdlOX/qKMvyxx/5N1m2bWutDTbs+eP65Rgpw5UdsCRJomx9Qyi0vhU7DsFZVGJRiWVSFv6BphjlQztOzlKMCqEdF9BQjNqgHRc5UowKoR0XolOMCqG5WYhiVAcdbzFGeS947Lcoc9M90PEXIw9GATpUMfKon0DFyMOrAhUj0IGKkalDVTEqgXY8BJYjRoXQjgd1c8SoDdrxMoWVF6NOaMcLb1ZbjJqheSnZGotR0E33bwovjgyUNE39Yrzf7wu+P1nt1PGsGHm5b6BiDP9+X/1z9LPJuu/7kA+e1j9H+5O1v1PXdS30pnsF+fz8BDrQTj19HoZhqc1QPkd/nytDjnFFc7ScAA000ARooIGGAGigCdBAA02ABpoADTTQBGigCdBAA02ABpoADTTQBGigCdBAA02ABpoADTTQBGigCdBAA02ABpoADTQhhBAiLl+hY8Jhdrbh8wAAAABJRU5ErkJggg==",
                fileName="modelica://TIL/Images/SplitterJoiner_left.png")}));
      end GasSplitterJoiner;
      annotation(classOrder={"HXInitialization","MassFlowJoiner","MassFlowSplitter","GasMassFlowJoiner","GasMassFlowSplitter","*","Testers"});
    end SplitterJoiner;
  annotation(classOrder={"Simple","FinAndTube","MPET","TubeAndTube","SplitterJoiner","Summaries","*","Internals"});
  end HeatExchangers;

  package OtherComponents
  "Compnent models for thermal, mechanical and controller models"
  extends TIL.Internals.ClassTypes.ComponentPackage;

    package Controllers "Controllers"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model PIController "PI-Controller with optional inputs for gain scheduling"

        /******************** Connectors *****************************/

        Modelica.Blocks.Interfaces.RealInput u_s "Connector of setpoint input signal"
          annotation (Placement(transformation(extent={{-74,-10},{-54,10}}, rotation=
                  0), iconTransformation(extent={{-66,-10},{-46,10}})));
        Modelica.Blocks.Interfaces.RealInput u_m
          "Connector of measurement input signal" annotation (Placement(
              transformation(
              origin={0,-64},
              extent={{10,-10},{-10,10}},
              rotation=270), iconTransformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,-58})));
        Modelica.Blocks.Interfaces.RealOutput y(start=yInitial)
          "Connector of actuator output signal"
          annotation (Placement(transformation(extent={{54,-10},{74,10}}, rotation=0),
              iconTransformation(extent={{54,-10},{74,10}})));
        Modelica.Blocks.Interfaces.BooleanInput activeInput if use_activeInput
          "true, if controller is on" annotation (Placement(transformation(extent={{
                  -74,-50},{-54,-30}}, rotation=0), iconTransformation(extent={{-66,-48},
                  {-46,-28}})));

        /*************************************************/

        Real u "Differenz between setpoint and measurement aka control error";

      public
        parameter String controllerType="P" "Controller Type" annotation (choices(
              choice="P" "P controller", choice="PI" "PI controller",  choice="I"
              "I controller"),                                                                     dialog(group=
                "Settings"));

        parameter Boolean invertFeedback=false "true, if feedback signal is inverted"
          annotation (dialog(group="Settings"));

        parameter Real offset = 0.0 "Operating point, added to proportional output"
          annotation (dialog(enable=(controllerType=="P"), group="Settings"));

        parameter String integralParameterType="Ti"
          "Controller Structure [k*(u + 1/Ti*integral(u))] or [k*u + ki*integral(u)]"
          annotation (choices(choice="Ti", choice="ki"), dialog(enable=(
                (controllerType == "PI") or (controllerType == "I")), group="Settings"));

        parameter Real k=1 "Proportional gain of controller"
          annotation (Dialog(enable=(not use_kInput) and (not (controllerType == "I" and integralParameterType == "ki")),  group="Setting parameters"));

        parameter SI.Time Ti(min=Modelica.Constants.small) = 0.5
          "Time constant of Integrator block" annotation (Dialog(enable=(not
                use_TiInput and controllerType <> "P" and integralParameterType ==
                "Ti"), group="Setting parameters"));

        parameter SI.DampingCoefficient ki(min=Modelica.Constants.small) = 0.5
          "Integral gain" annotation (Dialog(enable=(not use_kiInput and
                controllerType <> "P" and integralParameterType == "ki"), group=
                "Setting parameters"));

        parameter Boolean use_kInput=false "= true, if k defined by input"
          annotation (Dialog(enable= not (controllerType == "I" and integralParameterType == "ki"), tab="Advanced", group="Enable gain scheduling"));
       parameter Boolean use_TiInput=false "= true, if Ti defined by input"
          annotation (Dialog(enable=(controllerType <> "P" and integralParameterType
                 == "Ti"), tab="Advanced", group="Enable gain scheduling"));
        parameter Boolean use_kiInput=false "= true, if ki defined by input"
          annotation (Dialog(enable=(controllerType <> "P" and integralParameterType
                 == "ki"), tab="Advanced", group="Enable gain scheduling"));

        parameter Real yMax=1 "Upper limit of output"
          annotation (dialog(group="Limits"));
        parameter Real yMin=-yMax "Lower limit of output"
          annotation (dialog(group="Limits"));

        /***************** Initialization *************************/

        parameter String initType="initialOutput" "Type of initialization" annotation (
            choices(choice="zeroIntegralState", choice="initialOutput"),
            dialog(enable=controllerType<>"P", group="Initialization"));

        parameter Real yInitial=0 "Initial output of controller"
          annotation (dialog(enable=controllerType<>"P", group="Initialization"));

        /******************** Activation *****************************/
      public
        parameter Boolean use_activeInput=false
          "= true, if controller is switched on/off externally"
          annotation (dialog(group="Activation"));

        parameter Boolean use_y_notActive=false
          "= true, if output of not activated controller is defined externally. Otherwise output is hold at deactivation."
          annotation (dialog(group="Activation"));

         parameter SI.Time activationTime=0.0 "Time when controller is switched on"
          annotation (dialog(enable=not use_activeInput, group="Activation"));

      protected
        Modelica.Blocks.Routing.BooleanPassThrough getActive;
        Modelica.Blocks.Sources.BooleanExpression active_(y= time >=
              activationTime) if not use_activeInput;

        /**********************************************************/

      protected
        Real y_unlim;
        Real y_old;
        Real u_antiWindUp;
        Real integral;
        Real ki_internal = if integralParameterType == "Ti" then getInputs.k_in/getInputs.Ti_in else getInputs.ki_in;

        Internals.GetInputs getInputs
          annotation (Placement(transformation(extent={{-20,16},{0,36}})));

        Modelica.Blocks.Sources.Constant k_in_(k=k) if not use_kInput;
        Modelica.Blocks.Sources.Constant ki_in_(k=ki) if not use_kiInput;
        Modelica.Blocks.Sources.Constant Ti_in_(k=Ti) if not use_TiInput;
        Modelica.Blocks.Sources.RealExpression y_notActive_(y=y_old) if not use_y_notActive;

      public
        Modelica.Blocks.Interfaces.RealInput k_in if use_kInput
          "Connector of setpoint input signal" annotation (Placement(transformation(
                extent={{-74,30},{-54,50}}, rotation=0), iconTransformation(extent={{
                  -66,30},{-46,50}})));
        Modelica.Blocks.Interfaces.RealInput ki_in if use_kiInput
          "Connector of setpoint input signal" annotation (Placement(transformation(
                extent={{-74,10},{-54,30}},rotation=0), iconTransformation(extent={{-66,
                  10},{-46,30}})));
        Modelica.Blocks.Interfaces.RealInput Ti_in if use_TiInput
          "Connector of setpoint input signal" annotation (Placement(transformation(
                extent={{-74,10},{-54,30}},rotation=0), iconTransformation(extent={{-66,
                  10},{-46,30}})));

        Modelica.Blocks.Interfaces.RealInput y_notActive if use_y_notActive
          "Controller output if not active" annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,62}),              iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,58})));
      initial equation

        if initType == "zeroIntegralState" then
          integral = 0;
        else
          if controllerType == "P" then
            integral = 0.0;
          elseif controllerType == "PI" then
            integral = yInitial - getInputs.k_in*u;
          else
            integral = yInitial;
          end if;
        end if;

        y_old = yInitial;

      equation

        assert(yMax >= yMin, "PI controller output limits are not consistent: yMax (="
           + String(yMax) + ") < yMin (=" + String(yMin) + ")");

        assert((yInitial >= yMin) and (yInitial <= yMax) or (controllerType == "P"),
          "Output of PI controller yInitial must be between limits.");

        if invertFeedback then
          u = (u_m - u_s);
        else
          u = (u_s - u_m);
        end if;

        if controllerType == "P" then
          der(integral) = 0;
          y_unlim = getInputs.k_in*u + offset;
        elseif controllerType == "I" then
          der(integral) = if getActive.y then ki_internal*(u + u_antiWindUp) else 0.0;
          y_unlim = integral;
        else
          der(integral) = if getActive.y then ki_internal*(u + u_antiWindUp) else 0.0;
          y_unlim = getInputs.k_in*u + integral;
        end if;

        u_antiWindUp = if controllerType == "PI" then (y - y_unlim)/getInputs.k_in else (y - y_unlim);

      //__________ Activation _________________________

        when not getActive.y and not initial() then
          y_old = pre(y);
        end when;

        when getActive.y then
          if controllerType == "PI" then
             reinit(integral, pre(y) - getInputs.k_in*u);
          elseif controllerType == "I" then
             reinit(integral, pre(y));
          end if;
        end when;

        if getActive.y then
          y = smooth(0, if y_unlim > yMax then yMax else if y_unlim < yMin then yMin else y_unlim);
        else
          y = getInputs.y_notActive;
        end if;

      //__________ Connections _________________________

        connect(activeInput, getActive.u);
        connect(active_.y, getActive.u);

        connect(k_in_.y, getInputs.k_in);
        connect(ki_in_.y, getInputs.ki_in);
        connect(Ti_in_.y, getInputs.Ti_in);
        connect(y_notActive_.y, getInputs.y_notActive);

        connect(k_in, getInputs.k_in) annotation (Line(
            points={{-64,40},{-40,40},{-40,26},{-22,26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ki_in, getInputs.ki_in) annotation (Line(
            points={{-64,20},{-44,20},{-44,18},{-22,18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(Ti_in, getInputs.Ti_in) annotation (Line(
            points={{-64,20},{-40,20},{-40,22},{-22,22}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(y_notActive, getInputs.y_notActive) annotation (Line(points={{0,62},{0,
                46},{-26,46},{-26,34},{-22,34}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-60,-60},{60,60}}),
              graphics={Bitmap(
                extent={{-60,-60},{60,60}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAHgAAAB4CAIAAAC2BqGFAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAyklEQVR42u3SsQ3AIAxEUZwhvP98XsIpoUBUKErxfnft040hSfprsY6qInKxzNxDdzedmy+Oyfvg+CbQoEELNGjQCECDFmjQoAUatECDBi3QoAUaNGiBBi3QoEELNGiBBg1aoEELNGjQAg1aoEGDFmjQAg0atECDFmjQoAUatECDBi3QoAUaNGiBBi3QoEELNGiBBg1aoEELNGjQAg1aoEGDFmjQAg0atECDFmjQoAUatECDBi3QoAUaNGiBBi3QoEELNGiBliTp2AvV3wbfxPDYfgAAAABJRU5ErkJggg==",
                fileName="modelica://TIL/Images/PI_Controller_Icon.png"),
                                                                     Text(
                extent={{-60,30},{60,-30}},
                lineColor={0,0,0},
                textString="%controllerType")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}}),
                         graphics),
          Documentation(info="<html>
        <p>
        This controller can be used as P- or PI-controller.
        The controller can be switched on and off during the simulation.
        The output can be limited and an anti-wind-up is included.
        The initialisation can be steady state: der(y)=0 or with an initial output: y=yInitial.
        </p>
        <hr>
        </html>"));
      end PIController;

      package Internals
      extends TIL.Internals.ClassTypes.InternalPackage;

        model GetInputs "Get enabled inputs and parameters of disabled inputs"
          extends Modelica.Blocks.Interfaces.BlockIcon;

          Modelica.Blocks.Interfaces.RealInput ki_in
            annotation (Placement(transformation(extent={{-140,-100},{-100,-60}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput k_in
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}},rotation=
                   0)));
          Modelica.Blocks.Interfaces.RealInput Ti_in
             annotation (Placement(
                transformation(extent={{-140,-60},{-100,-20}},rotation=0)));
          Modelica.Blocks.Interfaces.RealInput y_notActive annotation (Placement(
                transformation(extent={{-140,60},{-100,100}}, rotation=0)));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}})));
        end GetInputs;
      annotation (classOrder={"BoundaryType","PartialMoistAirBoundary","GetInputs",
            "FlowAndHumidityDimensionSwitch","*"});
      end Internals;
    end Controllers;

    package Mechanical "Mechanical"
     extends TIL.Internals.ClassTypes.ComponentPackage;

      model RotatoryBoundary
        "Mechanical boundary element with optional type choice and Inputs"

        extends
        TIL.OtherComponents.Mechanical.BaseClasses.PartialRotatoryBoundary;

        parameter TIL.OtherComponents.Mechanical.Internals.BoundaryType boundaryType=
            "n" "Boundary type"                                                                         annotation(Dialog(group="Settings"));

        parameter Boolean use_wInput = false "true, if w is defined by input" annotation(Dialog(group="Angular Frequency",
          enable=(boundaryType=="w" or boundaryType=="w, tau")));

        parameter SI.AngularFrequency wFixed = 0.0 "Fixed Value for w (rad/s)"
                                                                       annotation(Dialog(group="Angular Frequency",
          enable=(boundaryType=="w" and not use_wInput or boundaryType=="w, tau" and not use_wInput)));

        parameter Boolean use_nInput = false "= true, if n is defined by input" annotation(Dialog(group="Speed",
          enable=(boundaryType=="n" or boundaryType=="n, tau")));

        parameter SI.Frequency nFixed = 0.0 "Fixed Value for n (rps)"                   annotation(Dialog(group="Speed",
          enable=(boundaryType=="n" and not use_nInput or boundaryType=="n, tau" and not use_nInput)));

        parameter Boolean use_tauInput = false
          "= true, if torque is defined by input"                                           annotation(Dialog(group="Torque",
          enable=(boundaryType=="tau" or boundaryType=="n, tau" or boundaryType=="w, tau")));

        parameter SI.Torque tauFixed = 0.0 "Fixed Value for torque" annotation(Dialog(group="Torque",
          enable=(boundaryType=="tau" and not use_tauInput or boundaryType=="n, tau" and not use_tauInput or boundaryType=="w, tau" and not use_tauInput)));

        Modelica.Blocks.Interfaces.RealInput n_in if                                           use_nInput
          "Prescribed boundary n [Hz]" annotation (Placement(transformation(extent={{-80,
                  10},{-60,30}}, rotation=0), iconTransformation(extent={{-50,-10},{
                  -30,10}})));
        Modelica.Blocks.Interfaces.RealInput tau_in if                                         use_tauInput
          "Prescribed boundary torque [N.m]"  annotation (Placement(transformation(extent={
                  {-80,-30},{-60,-10}}, rotation=0), iconTransformation(extent={{-50,-50},
                  {-30,-30}})));
        Modelica.Blocks.Interfaces.RealInput w_in if                                                   use_wInput
          "Prescribed boundary angular frequency [rad/s]"
          annotation (Placement(transformation(
                extent={{-80,50},{-60,70}}, rotation=0), iconTransformation(extent={{-50,30},
                  {-30,50}})));

      protected
        Internals.GetInputs getInputs
          annotation (Placement(transformation(extent={{-30,28},{-10,48}}, rotation=0)));

        Modelica.Blocks.Interfaces.RealInput n_in_=0.0 if not use_nInput;
        Modelica.Blocks.Interfaces.RealInput tau_in_=0.0 if not use_tauInput;
        Modelica.Blocks.Interfaces.RealInput w_in_=0.0 if not use_wInput;

      equation
        if (boundaryType == "n") then
          if (use_nInput) then
            n = getInputs. n_in;
          else
            n = nFixed;
          end if;
        elseif (boundaryType == "tau") then
          if (use_tauInput) then
            tau = getInputs. tau_in;
          else
            tau = tauFixed;
          end if;
        else
          if (use_wInput) then
            w = getInputs. w_in;
          else
            w = wFixed;
          end if;
        end if;

        w = 2*PI*n;
        der(rotatoryFlange.phi) = w;
        rotatoryFlange.tau = tau;

        connect(w_in, getInputs.w_in)
          annotation (Line(points={{-70,60},{-48,60},{-48,46},{-32,46}}, color={0,0,
                127}));
        connect(n_in, getInputs.n_in) annotation (Line(points={{-70,20},{-48,20},{-48,
                42},{-32,42}}, color={0,0,127}));
        connect(tau_in, getInputs.tau_in) annotation (Line(points={{-70,-20},{
                -44,-20},{-44,38},{-32,38}}, color={0,0,127}));

        connect(n_in_, getInputs.n_in);
        connect(w_in_, getInputs.w_in);
        connect(tau_in_, getInputs.tau_in);

       annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-100},
                  {40,100}}),
                        graphics={Text(
                extent={{-100,-70},{100,-110}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                textString=
                     "%boundaryType")}),Diagram(graphics));
      end RotatoryBoundary;

      package BaseClasses
      import TIL;
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialRotatoryBoundary "Partial rotatory boundary"

         TIL.Connectors.RotatoryFlange rotatoryFlange(phi(start=phiInitial,fixed=fixedPhiInitial)) annotation (Placement(
                transformation(extent={{-10,10},{10,30}}, rotation=0),
                iconTransformation(extent={{-10,-10},{10,10}})));

         parameter SI.Angle phiInitial = 0 "Initial value for rotatory flange angle"
           annotation(Dialog(tab="Advanced Initialization"));
         parameter Boolean fixedPhiInitial = true
            "if true, force usage of initial value phiInitial"
           annotation(Dialog(tab="Advanced Initialization"));

        protected
         SI.Frequency n "Frequency (rps)";
         SI.AngularFrequency w "Angular frequency";
         SI.Torque tau "Torque";

          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-100},
                    {40,100}}),
                           graphics={Rectangle(
                  extent={{-2,50},{2,-50}},
                  lineColor={135,135,135},
                  fillColor={135,135,135},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                  preserveAspectRatio=true, extent={{-40,-100},{40,100}})));

        end PartialRotatoryBoundary;
      end BaseClasses;

      package Internals
      extends TIL.Internals.ClassTypes.InternalPackage;

        type BoundaryType "Type choices for mechanical rotatory boundary"
        extends String;
          annotation(choices(
            choice="w" "w - Angular frequency is fixed",
            choice="n" "n - Speed is fixed",
            choice="tau" "tau - Torque is fixed"));
        end BoundaryType;

        model GetInputs "Get enable inputs and parameter of disabled inputs"
            extends Modelica.Blocks.Interfaces.BlockIcon;

            Modelica.Blocks.Interfaces.RealInput tau_in "Prescribed boundary torque"
                                         annotation (Placement(transformation(extent={{
                    -140,-20},{-100,20}}, rotation=0)));
            Modelica.Blocks.Interfaces.RealInput n_in "Prescribed boundary frequency"
                                                    annotation (Placement(
                transformation(extent={{-140,20},{-100,60}}, rotation=0)));
            Modelica.Blocks.Interfaces.RealInput w_in
            "Prescribed boundary angular frequency"
                                                   annotation (Placement(transformation(
                  extent={{-140,60},{-100,100}}, rotation=0)));

          annotation (Diagram(graphics));
        end GetInputs;
      annotation(classOrder = {"BoundaryType","PartialRotatoryBoundary", "GetInputsRotatoryBoundary", "*"});
      end Internals;
    annotation(classOrder={"RotatoryBoundary","RotatoryBoundaryWithInputs","RotatoryBoundaryWithInputsAndOutputs","Sensor_phi","Sensor_tau","Sensor_P","*","BaseClasses","Internals","Testers"});
    end Mechanical;

    package Thermal "Thermal"
    extends TIL.Internals.ClassTypes.ComponentPackage;

      model HeatBoundary "Boundary element with optional inputs"
        extends TIL.OtherComponents.Thermal.BaseClasses.PartialHeatBoundary;

        parameter Boolean use_heatFlowRateInput=false
          "= true, if heatFlow defined by input"
          annotation(Dialog(enable=(boundaryType=="Q_flow" or boundaryType=="T, Q_flow"), group="Heat Flow Rate"));
        parameter Boolean use_temperatureInput=false "= true, if T defined by input"
          annotation(Dialog(enable=(boundaryType=="T" or boundaryType=="T, Q_flow"), group="Temperature"));

        parameter SI.Temperature TFixed = 298.15 "Fixed temperature"
         annotation(Dialog(enable=(not use_temperatureInput and (boundaryType=="T" or boundaryType=="T, Q_flow")), group="Temperature"));

        parameter SI.HeatFlowRate Q_flowFixed = 0 "Fixed heat flow rate"
         annotation(Evaluate=true,Dialog(enable=(not use_heatFlowRateInput and (boundaryType=="Q_flow" or boundaryType=="T, Q_flow")), group="Heat Flow Rate"));

       Modelica.Blocks.Interfaces.RealInput T_in if    use_temperatureInput
          "Prescribed boundary temperature [K]"
          annotation (Placement(transformation(extent={{-100,50},{-80,70}}, rotation=
                  0), iconTransformation(extent={{-50,30},{-30,50}})));
       Modelica.Blocks.Interfaces.RealInput Q_flow_in if
                                                        use_heatFlowRateInput
          "Prescribed boundary heat flow rate [W]"
          annotation (Placement(transformation(extent={{-100,-10},{-80,10}}, rotation=
                 0), iconTransformation(extent={{-50,-30},{-30,-10}})));
      protected
        model GetInputs "Get enabled inputs and parameters of disabled inputs"
          extends Modelica.Blocks.Interfaces.BlockIcon;
          Modelica.Blocks.Interfaces.RealInput T_in "Prescribed boundary temperature"
            annotation (Placement(transformation(extent={{-140,60},{-100,100}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput Q_flow_in
            "Prescribed boundary heat flow rate"
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}},
                  rotation=0)));
          annotation (Diagram(graphics));
        end GetInputs;

      protected
        GetInputs getInputs
          annotation (Placement(transformation(extent={{-40,30},{-20,50}}, rotation=0)));

        Modelica.Blocks.Interfaces.RealInput T_in_=0 if not use_temperatureInput;
        Modelica.Blocks.Interfaces.RealInput Q_flow_in_=0 if not use_heatFlowRateInput;

      equation
        if boundaryType == "T" then
          if (use_temperatureInput) then
            temperature = getInputs.T_in;
          else
            temperature = TFixed;
          end if;
        else
          if (use_heatFlowRateInput) then
            heatFlowRate = getInputs.Q_flow_in;
          else
            heatFlowRate = Q_flowFixed;
          end if;
        end if;

        heatPort.T = temperature;
        heatPort.Q_flow = heatFlowRate;

        connect(getInputs.Q_flow_in, Q_flow_in_);
        connect(getInputs.T_in, T_in_);

        connect(T_in, getInputs.T_in) annotation (Line(points={{-90,60},{-60,60},{-60,
                48},{-42,48}}, color={0,0,127}));
        connect(Q_flow_in, getInputs.Q_flow_in)
                                              annotation (Line(points={{-90,0},{-60,0},
                {-60,40},{-42,40}}, color={0,0,127}));
        annotation (Diagram(graphics),
                             Icon(coordinateSystem(preserveAspectRatio=true, extent={
                  {-40,-60},{40,60}}),
                                  graphics={Text(
                extent={{-100,-70},{100,-110}},
                lineColor={0,0,0},
                textString=
                     "%boundaryType")}),
            Documentation(info="<html>
        <p>
        With boundary models you can determine the conditions at the boundary of your system.
        The heat boundary defines the temperature or heat flow.
        Boundaries have an egoistic notation of flow variables. 
        A negative heat flow rate leaves the boundary and a positive heat flow rate goes into the boundary.
        </p>
        <hr>
        </html>"));
      end HeatBoundary;

      package BaseClasses
      import TIL;
      extends TIL.Internals.ClassTypes.ModelPackage;

        partial model PartialHeatBoundary "Partial heat boundary"
          parameter TIL.OtherComponents.Thermal.Internals.BoundaryType boundaryType="T"
            "|Settings|Boundary type";

          TIL.Connectors.HeatPort heatPort "Heat port"
            annotation (Placement(transformation(extent={{-10,10},{10,30}}, rotation=0),
                iconTransformation(extent={{-10,-10},{10,10}})));

        protected
          SI.Temperature temperature "temperature";
          SI.HeatFlowRate heatFlowRate "heat flow rate";

          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-40,-60},
                    {40,60}}),
                           graphics={Rectangle(
                  extent={{-2,50},{2,-50}},
                  lineColor={204,0,0},
                  fillColor={204,0,0},
                  fillPattern=FillPattern.Solid)}));

        end PartialHeatBoundary;
      end BaseClasses;

      package Internals
       extends TIL.Internals.ClassTypes.InternalPackage;

        type BoundaryType
          extends String;
          annotation(choices(
            choice="T" "T - Temperature is fixed",
            choice="Q_flow" "Q_flow - Heat flow rate is fixed"));
        end BoundaryType;
      annotation(classOrder={"BoundaryType","PartialHeatBoundary","PartialMultiPortHeatBoundary","*"});
      end Internals;
     annotation(classOrder={"HeatBoundary","HeatBoundaryWithInputs","MultiPortHeatBoundary","MultiPortHeatBoundaryWithInputs","MultiPortHeatBoundaryWithInputArray","Sensor_T","MultiPortSensor_T","Sensor_Q_flow","MultiPortSensor_Q_flow","*","BaseClasses","Internals","Testers"});
    end Thermal;
    annotation(classOrder={"Thermal","*"});
  end OtherComponents;

  package Cells
  "Internal cell models of component models like tubes and heat exchangers"
   extends TIL.Internals.ClassTypes.ModelPackage;

    model GasCell "Fin efficiency disabled"

    /*********************** SIM ***********************************/

      inner parameter TILMedia.GasTypes.BaseGas gasType "Gas type"
                   annotation (Dialog(tab="SIM",group="SIM"),choices(
        choice=sim.gasType1 "Gas 1 as defined in SIM",
        choice=sim.gasType2 "Gas 2 as defined in SIM"));

    /****************** Connectors *******************/

      TIL.Connectors.GasPort portA(
        m_flow,
        final gasType = gasType) "Gas port A"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=
               0)));
      TIL.Connectors.GasPort portB(
        m_flow,
        final gasType = gasType) "Gas port B"
        annotation (Placement(transformation(extent={{90,-10},{110,10}}, rotation=0)));

      TIL.Connectors.HeatPort heatPort(T(start=273.15)) "Heat port"
        annotation (Placement(transformation(extent={{-10,88},{10,108}}, rotation=0)));

     /****************** General parameters *******************/

      inner input TIL.Cells.Geometry.FluidCellGeometry cellGeometry;

      parameter TIL.Internals.CellFlowType cellFlowType = "allow reverse flow";

     /******************* Fin ******************/

      replaceable model FinEfficiencyModel =
          TIL.Cells.TransportPhenomena.PartialFinEfficiency;

      FinEfficiencyModel finEfficiency annotation (Placement(transformation(extent=
                {{-10,2},{10,22}}, rotation=0)));

      replaceable model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum
        constrainedby TILMedia.SolidTypes.BaseSolid "Solid fluid type";

      inner TILMedia.Solid finMaterial(final T = heatPort.T,
        redeclare final model SolidType = FinMaterial)                                  annotation (Placement(
            transformation(extent={{10,-10},{30,10}}, rotation=0)));

      inner SI.CoefficientOfHeatTransfer alpha "Coefficient of heat transfer";

     /****************** Gas *******************/

      final parameter Boolean computeTransportProperties = heatTransfer.computeTransportProperties or pressureDrop.computeTransportProperties;

      TILMedia.Gas_ph gas_inStream(final p = if ((cellFlowType == "flow A-B") or ((cellFlowType <> "flow B-A") and (portA.m_flow >= 0))) then portA.p else portB.p,
        final h = if ((cellFlowType == "flow A-B") or ((cellFlowType <> "flow B-A") and (portA.m_flow >= 0))) then inStream(portA.h_outflow) else inStream(portB.h_outflow),
        final xi = if ((cellFlowType == "flow A-B") or ((cellFlowType <> "flow B-A") and (portA.m_flow >= 0))) then inStream(portA.xi_outflow) else inStream(portB.xi_outflow),
        final gasType = gasType,
        final computeTransportProperties = computeTransportProperties)
        "Properties of instreaming gas"
         annotation (Placement(transformation(extent={{-10,-72},{10,-52}}, rotation=
               0)));

     /****************** Heat transfer and pressure drop *******************/

      replaceable model HeatTransferModel =
          TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer
        constrainedby TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer
        "Gas heat transfer model";
      HeatTransferModel heatTransfer "Heat transfer"
        annotation (Placement(transformation(extent={{-10,40},{10,60}}, rotation=0)));

      replaceable model PressureDropModel =
          TIL.Cells.TransportPhenomena.PartialPressureDrop
        constrainedby TIL.Cells.TransportPhenomena.PartialPressureDrop
        "Gas pressure drop model";
      PressureDropModel pressureDrop "Pressure drop"
        annotation (Placement(transformation(extent={{-10,-40},{10,-20}}, rotation=
                0)));

      parameter Boolean useFalseDynamics=false
        "Sets pressure drop as differential state; True, if false dynamics should be used"
        annotation (Dialog(group="False Dynamics for Pressure Drop", tab="Advanced"));
      parameter Real falseDynamicsTimeConstant=0.1
        "Time constant of false dynamics for pressureDropState"
        annotation (Dialog(group="False Dynamics for Pressure Drop", tab="Advanced"));

      SI.Pressure pressureDropState(final start=pressureDropInitial, fixed=
            fixedPressureDropInitial) "Pressure drop used in false dynamics model";

      inner SI.MassFlowRate mdotHydraulic "Hydraulic mass flow rate";

      inner SI.HeatFlowRate QdotHydraulic(start=0) "Hydraulic heat flow rate";

      inner SI.Temperature wallTemperature "Wall temperature";

      SI.ThermalConductance overallAlphaA
        "alphaA value for cell, based on heat transfer and fin efficiency models";
      SI.ThermalConductance effAlphaA
        "Corrected alphaA based on exponential T distribution";

      /****************** Start and initial values *******************/

       parameter SI.Pressure pressureDropInitial = 0
        "Initial value for pressure drop"
        annotation (Dialog(group="Initialization", tab="Start and Initialization"));
      parameter Boolean fixedPressureDropInitial = false
        "If true, force usage of initial value"
        annotation (Dialog(group="Initialization", tab="Start and Initialization"));

      /****************** Additional inner objects *******************/

      inner TILMedia.Internals.PropertyRecord properties(
        final d=gas_inStream.d,
        final T=gas_inStream.T,
        final s=gas_inStream.s,
        final p=gas_inStream.p,
        final h=gas_inStream.h,
        final cp=gas_inStream.cp,
        final transp=gas_inStream.transp);

    protected
      final parameter Real linearLimitRatio=0.05;
      SI.MassFlowRate mdotLimit;
    equation
      portA.xi_outflow = inStream(portB.xi_outflow);
      portB.xi_outflow = inStream(portA.xi_outflow);

      if ((cellFlowType == "flow A-B") or ((cellFlowType <> "flow B-A") and (portA.m_flow >= 0))) then
        portA.h_outflow=0;
        inStream(portA.h_outflow)*portA.m_flow + portB.h_outflow*portB.m_flow + heatPort.Q_flow = 0.0
          "Energy balance";
      else
        portA.h_outflow*portA.m_flow + inStream(portB.h_outflow)*portB.m_flow + heatPort.Q_flow = 0.0
          "Energy balance";
        portB.h_outflow=0;
      end if;

      // Set false dynamics model   "Momentum balance"
      if (useFalseDynamics) then
        portA.p - portB.p = pressureDropState;
        der(pressureDropState) = (pressureDrop.pressureDrop - pressureDropState)/
          falseDynamicsTimeConstant;
      else
        pressureDropState = pressureDrop.pressureDrop;
        portA.p - portB.p = pressureDrop.pressureDrop;
      end if;

      portA.m_flow + portB.m_flow = 0.0 "Mass balance";

      mdotHydraulic = portA.m_flow/cellGeometry.nParallelHydraulicFlows;

      QdotHydraulic = heatPort.Q_flow/cellGeometry.nParallelHydraulicFlows;
      wallTemperature = heatPort.T;

      overallAlphaA = heatTransfer.alphaA * (1 + cellGeometry.finHeatTransferAreaRatio*(finEfficiency.eta - 1));
      alpha = heatTransfer.alphaA / cellGeometry.heatTransferArea;

      if noEvent(portA.m_flow >= mdotLimit) then
        effAlphaA = portA.m_flow*properties.cp*(1 - exp(-overallAlphaA/(portA.m_flow*properties.cp)));
      elseif noEvent(portA.m_flow >= 0) then
        effAlphaA = portA.m_flow*properties.cp*(1 - exp(-overallAlphaA/(mdotLimit*properties.cp)));
      elseif noEvent(portA.m_flow >= -mdotLimit) then
        effAlphaA = -portA.m_flow*properties.cp*(1 - exp(-overallAlphaA/(mdotLimit*properties.cp)));
      else
        effAlphaA = -portA.m_flow*properties.cp*(1 - exp(-overallAlphaA/(-portA.m_flow*properties.cp)));
      end if "Correction of heat transfer, see eqn. 137 of PhD Thesis Tegethoff";

      heatPort.Q_flow = effAlphaA*(heatPort.T - properties.T);
      mdotLimit=linearLimitRatio*overallAlphaA/properties.cp;

      if (cellFlowType == "flow A-B") then
        assert(portA.m_flow > 0, "The gas cell was set to flow A-B but the mass flow rate at port A is negative!");
        assert(portB.m_flow < 0, "The gas cell was set to flow A-B but the mass flow rate at port B is positive!");
      elseif (cellFlowType == "flow B-A") then
        assert(portA.m_flow < 0, "The gas cell was set to flow B-A but the mass flow rate at port A is negative!");
        assert(portB.m_flow > 0, "The gas cell was set to flow B-A but the mass flow rate at port B is positive!");
      end if;

      annotation (Icon(graphics={Bitmap(
              extent={{-100,-100},{100,100}},
              imageSource=
                  "iVBORw0KGgoAAAANSUhEUgAAAMgAAADICAIAAAAiOjnJAAAACXBIWXMAAAsTAAALEwEAmpwYAAABhElEQVR42u3UsQ3AMAhFQZPJ2JzRSEvhynKkFHcd7dcTawEAcFvMo6oswpnMjIh9WN1tIM6/1AjrMQdfEBbCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAuEhbAQFggLYSEsEBbCQlggLISFsEBYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAuEhbAQFggLYSEsEBbCQlggLISFsEBYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYSEsEyAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLAAD4qReb+Qd/FUGUbAAAAABJRU5ErkJggg==",
              fileName="modelica://TIL/Images/CellUni.png"),
                                       Text(
              extent={{-80,40},{80,-40}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString=
                   "Gas")}),Documentation(info="<html>
        <a href=\"../Documentation/effectiveheattransfer.pdf\">Effective heat transfer coefficient relating to the inlet temperature</a>
        </html>"),Diagram(graphics));
    end GasCell;

    model MoistAirWallCell
      "Moist Air Cell including dynamic mass and energy balance for water film"

    /*********************** SIM ***********************************/

      inner parameter TILMedia.GasTypes.VDIWA_MoistAir_nc3 gasType "Gas type"
                   annotation (Dialog(tab="SIM",group="SIM"),choices(
        choice=sim.gasType1 "Gas 1 as defined in SIM",
        choice=sim.gasType2 "Gas 2 as defined in SIM"));

      /****************** Connectors *******************/

      TIL.Connectors.HeatPort wallHeatPortN(T(start=TWallInitial))
        "Heat port north"
        annotation (Placement(transformation(extent={{-10,90},{10,110}}, rotation=0)));
      TIL.Connectors.HeatPort wallHeatPortW(T(start=TWallInitial)) "Heat port west"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}}, rotation=0)));
      TIL.Connectors.HeatPort wallHeatPortE(T(start=TWallInitial)) "Heat port east"
        annotation (Placement(transformation(extent={{90,50},{110,70}}, rotation=0)));
      TIL.Connectors.GasPort moistAirPortA(final gasType=gasType,
        m_flow( start=m_flowStart))    annotation (Placement(transformation(extent=
                {{-110,-70},{-90,-50}}, rotation=0)));
      TIL.Connectors.GasPort moistAirPortB(final gasType=gasType,
        m_flow( start=-m_flowStart))    annotation (Placement(transformation(extent=
               {{90,-70},{110,-50}}, rotation=0)));

       /****************** General parameters *******************/

      inner input TIL.Cells.Geometry.FluidCellGeometry cellGeometry;
      input TIL.Cells.Geometry.WallCellGeometry wallCellGeometry;
      parameter TIL.Internals.CellFlowType cellFlowType = "allow reverse flow"
         annotation(Dialog(group="Moist Air",tab="Advanced"));
      parameter Boolean flagActivateWallCondensation=true
        "if true, water condenses on the wall surface";
      parameter Boolean flagActivateDynWaterBalance = false
        "if true, the condensate on the wall surface is balanced dynamically"
         annotation(Dialog(group="Film",tab="Advanced"));

      /******************* Fin **********************************/

      replaceable model FinEfficiencyModel =
          TIL.Cells.TransportPhenomena.PartialFinEfficiency;

      FinEfficiencyModel finEfficiency annotation (Placement(transformation(extent=
                {{-10,-78},{10,-58}}, rotation=0)));

      replaceable model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum
        constrainedby TILMedia.SolidTypes.BaseSolid "Solid fluid type";

      inner TILMedia.Solid finMaterial(final T = filmSurfaceTemperature,
        redeclare final model SolidType = FinMaterial)                                  annotation (Placement(
            transformation(extent={{10,-10},{30,10}}, rotation=0)));

      inner SI.CoefficientOfHeatTransfer alpha
        "Coefficient of heat transfer of this cell";

      inner SI.PartialPressure p_s_Wall=moistAirWall.p_s
        "Saturation partial water pressure at wall";

     /****************** Heat transfer and pressure drop Moist Air side *******************/

      replaceable model MoistAirHeatTransferModel =
        TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer
        constrainedby TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer
        "Gas heat transfer model";
      MoistAirHeatTransferModel heatTransfer "moist air heat transfer"
        annotation (Placement(transformation(extent={{-48,-78},{-28,-58}}, rotation=
               0)));

      replaceable model MoistAirPressureDropModel =
        TIL.Cells.TransportPhenomena.PartialPressureDrop
        constrainedby TIL.Cells.TransportPhenomena.PartialPressureDrop
        "Gas pressure drop model";
      MoistAirPressureDropModel pressureDrop "Pressure drop"
        annotation (Placement(transformation(extent={{28,-78},{48,-58}}, rotation=0)));
      parameter Boolean useFalseDynamics=true
        "sets pressure drop as differential state; True, if false dynamics should be used"
      annotation(Dialog(group="False Dynamics for Pressure Drop",tab="Advanced"));
      parameter Real falseDynamicsTimeConstant=0.1
        "Time constant of false dynamics for pressureDropState" annotation(Dialog(group="False Dynamics for Pressure Drop",tab="Advanced"));
      SI.Pressure pressureDropState(final start=pressureDropMoistAirInitial, fixed = fixedPressureDropMoistAirInitial)
        "Pressure drop used in false dynamics model";

      inner SI.MassFlowRate mdotHydraulic(final start=m_flowStart/
            cellGeometry.nParallelHydraulicFlows) "Hydraulic mass flow rate";

      inner SI.HeatFlowRate QdotHydraulic(start=0) "Hydraulic heat flow rate";

      inner TILMedia.Internals.PropertyRecord properties = TILMedia.Internals.PropertyRecord(
        d = dMoistAirIn,
        T = TMoistAirIn,
        cp = cpMoistAirIn,
        s = sMoistAirIn,
        p = pMoistAirIn,
        h = hMoistAirIn,
        transp = transpMoistAirIn);

    protected
      final parameter Boolean computeTransportProperties = heatTransfer.computeTransportProperties or pressureDrop.computeTransportProperties;
      parameter Real linearLimitRatio=0.05;
      SI.MassFlowRate mdotLimit
        "below this mass flow the alpha_eff may cause div 0";

      /****************** MoistAir *******************/
    public
      input Real Le=1 "Lewis Factor" annotation(Dialog);
      TILMedia.Gas_ph   moistAir_inStream(
        final gasType = gasType,
        final p = pMoistAirIn,
        final h = hMoistAirIn,
        final xi = xiMoistAirIn[1:end-1],
        final computeTransportProperties=computeTransportProperties)
        annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}, rotation=
               0)));

      SI.MassFraction[gasType.nc-1] xiWall(start = gasType.xi_default)
        "Mass fraction vector at wall surface layer";

      TILMedia.Gas_pT   moistAirWall(
        final gasType = gasType,
        final p = 0.5*(moistAirPortA.p + moistAirPortB.p),
        final T(final start=TWallInitial) = filmSurfaceTemperature,
        final xi = xiWall,
        final computeTransportProperties = false)
        annotation (Placement(transformation(extent={{-30,-10},{-10,10}}, rotation=
                0)));

      /***************** Water film *********************************/

    public
      inner SI.Mass massFilm "mass of water/ice film";
      inner SI.SpecificEnthalpy hFilm "specific enthalpy of water/ice film";
      inner SI.MassFraction x "liquid film mass fraction (= 1 - ice mass fraction)";
      inner Real filmSurfaceWettingFraction
        "wetting fraction of wall and fin surface";
      SI.MassFlowRate mdotWaterDrain "condensate leaving heat exchanger";
      SI.MassFlowRate mdotCondensate;
      SI.MassFlowRate mdotEvaporate;

      inner parameter SI.Mass massFilmMax(final min=0)
        "maximum amount of water in steady state";
      inner parameter Real dryOutRatio(final min=0, final max=1) = 0.6
        "below this water ratio, wetting partial";

    protected
      SI.Enthalpy H_WallPlusIceFilm_0C "enthalpy of wall + ice at freezing point";
      SI.Enthalpy H_WallPlusLiquidFilm_0C
        "enthalpy of wall + liquid water at freezing point";
      SI.SpecificEnthalpy h_LiquidFilm_0C
        "enthalpy of liquid water at freezing point";
      SI.SpecificEnthalpy h_IceFilm_0C "enthalpy of ice at freezing point";
      SI.SpecificEnthalpy delta_hv "specific enthalpy of evaporation / sublimation";
      SI.Energy H_WallPlusFilm "enthalpy of wall and film";

      /****************** Start and initial values *******************/
    public
      parameter SI.MassFlowRate m_flowStart=0.5 "Start value for mass flow rate"
        annotation(Dialog(group="Start Values",tab="Start and Initialization"));

      parameter SI.Temperature TWallInitial "Initial wall temperature" annotation(Dialog(group="Initialization",tab="Start and Initialization"));
      parameter SI.Mass massFilmInitial(final min=0)
        "Initial value for water film mass"                                              annotation(Dialog(group="Initialization",tab="Start and Initialization"));
      parameter Boolean initWallTemperatureInSteadyState=false
        "if true, initialise wall temperature in steady state"                                                        annotation(Dialog(group="Initialization",tab="Start and Initialization"));
      parameter SI.Pressure pressureDropMoistAirInitial = 0
        "Start value for pressure drop"                                                     annotation(Dialog(group="Initialization",tab="Start and Initialization"));
      parameter Boolean fixedPressureDropMoistAirInitial = false
        "if true, force usage of initial value"                                                          annotation(Dialog(group="Initialization",tab="Start and Initialization"));

      /********************** Additional variables *********************/

      SI.ThermalConductance alphaA_apparent "fin efficiency corrected alphaA";
      Real betaA_eff "mass transfer coefficient";
      SI.ThermalConductance alphaA_eff
        "alphaA for inlet temperature difference instead of LMTD";

      SI.Density dMoistAirIn;
      SI.SpecificEnthalpy hMoistAirIn;
      SI.AbsolutePressure pMoistAirIn;
      SI.SpecificEntropy sMoistAirIn;
      SI.Temperature TMoistAirIn;
      SI.MassFraction xiCondensingMoistAirIn;
      SI.MassFraction[gasType.nc] xiMoistAirIn;
      SI.SpecificHeatCapacity cpMoistAirIn;
      SI.MassFlowRate mdotMoistAirIn;
      TILMedia.Internals.TransportPropertyRecord transpMoistAirIn;

     /****************** Wall Material *******************/

     TILMedia.Solid wallMaterial(redeclare final model SolidType =
            WallMaterial,
        T=wallStateTemperature) "Wall material in center of cell"  annotation (Placement(
            transformation(extent={{-10,40},{10,60}}, rotation=0)));

      replaceable model WallHeatTransferModel =
          TIL.Cells.TransportPhenomena.PartialSolidHeatTransfer
        constrainedby TIL.Cells.TransportPhenomena.PartialSolidHeatTransfer
        "Heat transfer model";

      replaceable model WallMaterial = TILMedia.SolidTypes.TILMedia_Aluminum
        constrainedby TILMedia.SolidTypes.BaseSolid "Solid fluid type";

      TransportModules.SolidHeatTransferModule  wallHeatTransfer(
          final properties=TILMedia.Internals.SolidPropertyRecord(
              d=wallMaterial.d,
              T=wallMaterial.T,
              cp=wallMaterial.cp,
              lambda=wallMaterial.lambda),
           final cellGeometry = wallCellGeometry,
           redeclare final WallHeatTransferModel heatTransfer)
        annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));

     /****************** Additional variables *******************/

      SI.Mass geometricWallMass = wallCellGeometry.volume*wallMaterial.d
        "Geometric mass";

      parameter TIL.Cells.Internals.WallCellStateType wallCellStateType=
                                            "state south";

      SI.HeatFlowRate QflowMoistAirToWall;
      SI.HeatFlowRate QflowMoistAirToWallLatent;
      SI.Temperature filmSurfaceTemperature(start=TWallInitial);
      SI.Temperature wallSurfaceTemperature(start=TWallInitial);
    protected
      SI.Temperature wallStateTemperature(start=TWallInitial);
      inner SI.Temperature wallTemperature=filmSurfaceTemperature;

      TILMedia.Gas_pT   moistAirFrozenFilm(
        final p=pMoistAirIn,
        final T=273.15,
        final gasType = gasType,
        final computeTransportProperties=false)
        annotation (Placement(transformation(extent={{-8,16},{12,36}}, rotation=0)));

    //Wikipedia:
    //  SI.ThermalConductivity lambda_ice= 2.33;

    //->"Lehrbuch der Kältetechnik":
      parameter SI.ThermalConductivity lambda_ice= 2.2
        "Thermal conductivity of ice";                                               //Standardeis
    //  SI.ThermalConductivity lambda_ice= 0.15 "Thermal conductivity of ice";//Reif
    //  SI.ThermalConductivity lambda_ice= 0.11 "Thermal conductivity of ice";//Schnee frisch
      parameter SI.ThermalConductivity lambda_liquidWater= 0.561
        "Thermal conductivity of liquid water";

      SI.ThermalConductivity lambda_water= (1-x2)*lambda_ice + x2*lambda_liquidWater
        "Thermal conductivity of water (liquid and solid)";

    //Diss Kosowski / Auracher Sahinagic eta al. 2004
      Real epsilon "Porosity (denstiy ratio of frost and ice)";
      parameter SI.Density rho_ice=920 "Density of ice";
      SI.Density rho_water=(1-x2)*rho_ice + x2*1000;
      parameter SI.Density rho_frost_min=epsilonFixed*rho_ice
        "Minimum density of frost";
      inner SI.Density rho_frost "Mean density in frost";
      Real c1=0.042 + 0.42*0.995^rho_frost "factor";
      SI.ThermalConductivity lambda_S = (1-epsilon)*transpMoistAirIn.lambda + epsilon*lambda_water
        "Serial thermal conductivity";
      SI.ThermalConductivity lambda_P = transpMoistAirIn.lambda * lambda_water / ((1-epsilon)*lambda_water + epsilon*transpMoistAirIn.lambda)
        "Parallel thermal conductivity";
      inner SI.ThermalConductivity lambda_CD = 1/(c1/lambda_P + (1-c1)/lambda_S)
        "thermal conductivity of frost";
      SI.ThermalResistance R_frost "Thermal resistance of frost";
      inner SI.Length filmHeight= massFilm/rho_frost/cellGeometry.heatTransferArea
        "Height of film/ice";
      SI.Mass massGasInIce(min=-1) "Mass of Gas stored in ice";
      Real x2=TIL.Utilities.Numerics.smoothTransition( (273.16-wallSurfaceTemperature)/max(filmSurfaceTemperature+0.01-wallSurfaceTemperature,0.01), 0.5, 1);

      input Real mDotGasPerMDotWater=moistAirFrozenFilm.d/rho_ice / (epsilonFixed)
        "Gas mass fraction in newly formed frost below freezing point";
    public
      parameter Real epsilonFixed=0.23;
      parameter String frostingModel="simple" annotation(choices(choice="dynamic", choice="simple"));
      parameter String waterHeatConductivity="simple" annotation(choices(choice="detailed", choice="simple"));

    initial equation
      if (not flagActivateDynWaterBalance) then
        H_WallPlusFilm=wallMaterial.cp*TWallInitial*geometricWallMass;
      else
        massFilm = massFilmInitial;
        if noEvent(TWallInitial>=273.16) then
          H_WallPlusFilm=wallMaterial.cp*TWallInitial*geometricWallMass + massFilm*(moistAirWall.h_i[gasType.condensingIndex]-moistAirWall.delta_hv);
        else
          H_WallPlusFilm=wallMaterial.cp*TWallInitial*geometricWallMass + massFilm*(moistAirWall.h_i[gasType.condensingIndex]-moistAirWall.delta_hd);
        end if;
      end if;

      if initWallTemperatureInSteadyState then
        der(H_WallPlusFilm) = 0;
      end if;

    equation
      mdotHydraulic = mdotMoistAirIn/cellGeometry.nParallelHydraulicFlows;
      QdotHydraulic = - QflowMoistAirToWall/cellGeometry.nParallelHydraulicFlows;

      if noEvent((massFilm/(massFilmMax)) > dryOutRatio) then
          filmSurfaceWettingFraction = 1;
      else
          filmSurfaceWettingFraction = 1/dryOutRatio*(massFilm/massFilmMax);
      end if;

    //------ Balance equations MoistAir ------

      0.0 = moistAirPortA.m_flow + moistAirPortB.m_flow - mdotCondensate + mdotEvaporate
        "mass balance moist air";

      if (cellFlowType=="flow A-B" or cellFlowType <> "flow B-A" and moistAirPortA.m_flow>0) then
        moistAirPortA.h_outflow=0;
        moistAirPortA.xi_outflow=zeros(gasType.nc-1);
        zeros(gasType.nc-1) = inStream(moistAirPortA.xi_outflow)*moistAirPortA.m_flow + (moistAirPortB.xi_outflow)*moistAirPortB.m_flow - cat(1, {mdotCondensate}, zeros(gasType.nc-2)) + cat(1, {mdotEvaporate}, zeros(gasType.nc-2))
          "condensing component mass balance moist air";
        0 = inStream(moistAirPortA.h_outflow)*moistAirPortA.m_flow + moistAirPortB.h_outflow*moistAirPortB.m_flow - QflowMoistAirToWall + hFilm*(mdotEvaporate - mdotCondensate)
          "energy balance moist air";
      else
        zeros(gasType.nc-1) = (moistAirPortA.xi_outflow)*moistAirPortA.m_flow + inStream(moistAirPortB.xi_outflow)*moistAirPortB.m_flow - cat(1, {mdotCondensate}, zeros(gasType.nc-2)) + cat(1, {mdotEvaporate}, zeros(gasType.nc-2))
          "condensing component water mass balance moist air";
        0 = moistAirPortA.h_outflow*moistAirPortA.m_flow + inStream(moistAirPortB.h_outflow)*moistAirPortB.m_flow - QflowMoistAirToWall + hFilm*(mdotEvaporate - mdotCondensate)
          "energy balance moist air";
        moistAirPortB.h_outflow=0;
        moistAirPortB.xi_outflow=zeros(gasType.nc-1);
      end if;

    // Set false dynamics model
      if (useFalseDynamics) then
        moistAirPortA.p - moistAirPortB.p = pressureDropState;
        der(pressureDropState) = (pressureDrop.pressureDrop - pressureDropState)/falseDynamicsTimeConstant;
      else
        pressureDropState = pressureDrop.pressureDrop;
        moistAirPortA.p - moistAirPortB.p = pressureDrop.pressureDrop;
      end if;

    //------ Heat flow rates MoistAir ------

      mdotLimit = linearLimitRatio*alphaA_apparent/cpMoistAirIn;

      alphaA_apparent = heatTransfer.alphaA * (1 + cellGeometry.finHeatTransferAreaRatio * (finEfficiency.eta - 1));
      alphaA_eff = mdotMoistAirIn * cpMoistAirIn * (1 - exp(-alphaA_apparent / noEvent(max(mdotMoistAirIn, mdotLimit)) / cpMoistAirIn));
      alpha = heatTransfer.alphaA / cellGeometry.heatTransferArea;

      QflowMoistAirToWallLatent = (mdotCondensate - mdotEvaporate)*delta_hv;
      QflowMoistAirToWall = alphaA_eff * (TMoistAirIn - filmSurfaceTemperature) + QflowMoistAirToWallLatent;
    //------ Mass flow rates ------

      if (flagActivateWallCondensation) then
        mdotCondensate = betaA_eff * dMoistAirIn * noEvent(max((xiCondensingMoistAirIn - moistAirWall.xi_s),0.0));
      else
        mdotCondensate = 0;
      end if;

      if (flagActivateDynWaterBalance) then
        mdotWaterDrain=mdotCondensate * massFilm/massFilmMax * TIL.Utilities.Numerics.smoothTransition( (273.16-wallSurfaceTemperature)/max(filmSurfaceTemperature+0.01-wallSurfaceTemperature,0.01), 0.5, 1)
          "PhD Boettcher, page 31";
        mdotEvaporate = if (noEvent(massFilm>1e-10)) then filmSurfaceWettingFraction * betaA_eff * dMoistAirIn * noEvent(max((moistAirWall.xi_s - xiCondensingMoistAirIn),0.0)) else 0;
      else
        mdotWaterDrain=mdotCondensate;
        mdotEvaporate = 0;
      end if;

      betaA_eff = alphaA_eff/cpMoistAirIn/dMoistAirIn*Le;
      assert(gasType.nc>1,"not enough mixture components");
      assert(gasType.condensingIndex>0,"no condensing component");
      assert(gasType.condensingIndex==1,"condensing index must be 1");
      moistAirWall.phi = 100;
      moistAirWall.xi_dryGas = moistAir_inStream.xi_dryGas;

      dMoistAirIn = moistAir_inStream.d;
      sMoistAirIn = moistAir_inStream.s;
      TMoistAirIn = moistAir_inStream.T;
      cpMoistAirIn = moistAir_inStream.cp;
      transpMoistAirIn = moistAir_inStream.transp;
      if (cellFlowType=="flow A-B"  or cellFlowType <> "flow B-A" and moistAirPortA.m_flow>0) then
        xiMoistAirIn = cat(1, inStream(moistAirPortA.xi_outflow), {1-sum(inStream(moistAirPortA.xi_outflow))});
        mdotMoistAirIn = moistAirPortA.m_flow;
        hMoistAirIn = inStream(moistAirPortA.h_outflow);
        pMoistAirIn = moistAirPortA.p;
      else
        xiMoistAirIn = cat(1, inStream(moistAirPortB.xi_outflow), {1-sum(inStream(moistAirPortB.xi_outflow))});
        mdotMoistAirIn = moistAirPortB.m_flow;
        hMoistAirIn = inStream(moistAirPortB.h_outflow);
        pMoistAirIn = moistAirPortB.p;
      end if;
      xiCondensingMoistAirIn = xiMoistAirIn[max(gasType.condensingIndex,1)];

    //------ Wall Cell heat conduction ------

      wallHeatPortW.Q_flow = (wallHeatPortW.T - wallMaterial.T)/wallHeatTransfer.R_WE*2.0;
      wallHeatPortE.Q_flow = (wallHeatPortE.T - wallMaterial.T)/wallHeatTransfer.R_WE*2.0;

    //------ Ice Buildup at constant Temperature ------
      h_LiquidFilm_0C = moistAirFrozenFilm.h_i[max(gasType.condensingIndex,1)]-moistAirFrozenFilm.delta_hv;
      h_IceFilm_0C = moistAirFrozenFilm.h_i[max(gasType.condensingIndex,1)]-moistAirFrozenFilm.delta_hd;
      H_WallPlusLiquidFilm_0C=wallMaterial.cp*(273.16-(filmSurfaceTemperature - wallMaterial.T))*geometricWallMass + massFilm*h_LiquidFilm_0C;
      H_WallPlusIceFilm_0C=wallMaterial.cp*(273.15-(filmSurfaceTemperature - wallMaterial.T))*geometricWallMass + massFilm*h_IceFilm_0C;
      if noEvent(H_WallPlusFilm<H_WallPlusIceFilm_0C) then
        x = 0;
    //    H_WallPlusFilm=wallMaterial.cp*wallMaterial.T*geometricWallMass + massFilm*hFilm;
        (H_WallPlusFilm-massFilm*hFilm)/(wallMaterial.cp*geometricWallMass)=wallMaterial.T;
        delta_hv = moistAirWall.delta_hd;
      elseif noEvent(H_WallPlusFilm<H_WallPlusLiquidFilm_0C) then
        x = TIL.Utilities.Numerics.smoothTransition(1- (H_WallPlusFilm-H_WallPlusIceFilm_0C)/(H_WallPlusLiquidFilm_0C-H_WallPlusIceFilm_0C), 0.5, 1);
        moistAirWall.T = x*0.01+273.15;//273.16;
        delta_hv = x*(moistAirFrozenFilm.delta_hv-moistAirFrozenFilm.delta_hd)+moistAirWall.delta_hd;
      else
        x = 1;
        (H_WallPlusFilm-massFilm*hFilm)/(wallMaterial.cp*geometricWallMass)=wallStateTemperature;
        delta_hv = moistAirWall.delta_hv;
      end if;
      hFilm = moistAirWall.h_i[max(gasType.condensingIndex,1)]-delta_hv;

      if (frostingModel=="dynamic") then
    //   rho_frost= 1/(1/moistAirFrozenFilm.d * massGasInIce/(massGasInIce + massFilm)  +  1/rho_water * massFilm/(massGasInIce + massFilm));
        rho_frost= min(max((massGasInIce + massFilm)/max(1e-12,1/moistAirFrozenFilm.d * massGasInIce  +  1/rho_water * massFilm),rho_frost_min),rho_water);
        der(massGasInIce) = mDotGasPerMDotWater*(mdotCondensate-mdotWaterDrain)*(1-x2) - 1*(massGasInIce)*x2;
      else
        epsilon=epsilonFixed;
        massGasInIce=-1;
      end if;
      epsilon=rho_frost/rho_water;
      if (waterHeatConductivity=="detailed") then
        R_frost=filmHeight/cellGeometry.heatTransferArea/lambda_CD;
      else
        R_frost=0;
      end if;
    //------ Mass and Energy Balance of combined Film and Wall Volume------
      if (flagActivateDynWaterBalance) then
        der(massFilm) = mdotCondensate - mdotEvaporate - mdotWaterDrain
          "mass balance water film";
      else
        massFilm = 0 "mass balance water film";
      end if;

      der(H_WallPlusFilm) = wallHeatPortN.Q_flow + wallHeatPortW.Q_flow + wallHeatPortE.Q_flow + QflowMoistAirToWall + hFilm*(mdotCondensate - mdotEvaporate-mdotWaterDrain)
        "energy balance wall cell plus film";

    //------ Differential State in Wall Cell ------

      if wallCellStateType == "state north" then
        wallMaterial.T = wallHeatPortN.T;
        QflowMoistAirToWall = (filmSurfaceTemperature - wallMaterial.T)/(wallHeatTransfer.R_NS+R_frost);
        QflowMoistAirToWall = (wallSurfaceTemperature - wallMaterial.T)/(wallHeatTransfer.R_NS);//calculate wallSurfaceTemperature
      elseif wallCellStateType == "state south" then
        wallMaterial.T = filmSurfaceTemperature;
        wallHeatPortN.Q_flow = (wallHeatPortN.T - wallMaterial.T)/(wallHeatTransfer.R_NS+R_frost);
        wallHeatPortN.Q_flow = (wallHeatPortN.T - wallSurfaceTemperature)/(wallHeatTransfer.R_NS);//calculate wallSurfaceTemperature
      elseif wallCellStateType == "no wall resistance" then
        wallMaterial.T = filmSurfaceTemperature;
        wallHeatPortN.T = filmSurfaceTemperature;
        wallHeatPortN.T = wallSurfaceTemperature;//calculate wallSurfaceTemperature
      else
        wallHeatPortN.Q_flow = (wallHeatPortN.T - wallMaterial.T)/(wallHeatTransfer.R_NS*2.0);
        QflowMoistAirToWall = (filmSurfaceTemperature - wallMaterial.T)/(wallHeatTransfer.R_NS*2.0+R_frost);
        QflowMoistAirToWall = (wallSurfaceTemperature - wallMaterial.T)/(wallHeatTransfer.R_NS*2.0);//calculate wallSurfaceTemperature
      end if;

      annotation (Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            initialScale=0.1), graphics),
                           Icon(
            coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            initialScale=0.1), graphics={Bitmap(extent={{-100,-100},{100,100}},
              imageSource=
                  "iVBORw0KGgoAAAANSUhEUgAAAMgAAADICAIAAAAiOjnJAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAsdJREFUeNrs3dFN40AUhlFCCTTibni1O3FKSB00kY6IU0EYKVIEEjOeO5M8JD7nAYGUH9DVtxLLZuHtDQCAe9v9fuN4POYetyzLNE3n83kYhsPhUPnerTa1yj7iknE6ndJHSg9IL9PrlzpWW1vFwnJHq8pVICx3tKpf1YbljlahVVVY7mgVXa2H5Y5WDauVsNzRqm1VCssdrZpX2bDc0apnlQ3LHa16VqVvz7ujVfNKVVYPWZW+xnJHq+ZV+B+h3dGqZtIb1jzP0c8yGcfR6rVXvWFd2wp9ltc/AelztXrhVfaJfit/Y4Tys0Z3O2EhLISFsISFsBAWwhIWwkJYCEtYCAthISxhISyEhbCEhbAQFsISFsJCWAhLWAgLYSEsYSEshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyExauG9f7vI5ZlmaYpvQy936dY7ff76L2sele3X18+DEN6cxzH0C89f5bVPM9Wj1iVwrot08v0evTjWW15lQ3LHa16Vtmw3NGqZ1X6MssdrZpXqrJ6yKr0NZY7WjWvVr7d4I5WbauusNzR6v5huaNVYdUYljtara7CYbmjVWNVhbDc0apmFQvLHa0qV4Gw3NGqflUbljtahVZVYbmjVXTlObTAkz7//c9bX98uQrvPj5X/TAGdhIWwEBbCAmEhLLbMDwXhfjH5aTMIC2EhLGEhLISFsISFsBAWwhIWwkJYCEtYCAthISxhISyEhbCEhbAQFsISFsJCWAhLWAgLYSEs10FYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAthCQthISyEJSyEhbAQFgAAD/EjwAAt90zn4TGK2QAAAABJRU5ErkJggg==",
              fileName="modelica://TIL/Images/MoistAirWallCell.png"),
            Text(
              extent={{-64,80},{70,26}},
              lineColor={0,0,0},
              textString="Wall"),
            Text(
              extent={{-70,-16},{78,-92}},
              lineColor={0,0,0},
              textString="MoistAir")},             fileName=
                  "modelica://TIL/Images/MoistAirWallCell.png"),
                                                   fileName =   "../Images/MoistAirWallCell.png");
    end MoistAirWallCell;

    model VLEFluidCell

      inner parameter TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType
        "VLE fluid type" annotation (Dialog(tab="SIM",group="SIM"),choices(
        choice=sim.vleFluidType1 "VLE fluid 1 as defined in SIM",
        choice=sim.vleFluidType2 "VLE fluid 2 as defined in SIM",
        choice=sim.vleFluidType3 "VLE fluid 3 as defined in SIM"));

     /****************** General parameters *******************/
      parameter Boolean useFalseDynamics=true
        "sets pressure drop as differential state; True, if false dynamics should be used" annotation(Dialog(group="Parameters", tab="Advanced"));
      parameter Boolean useAlphaAState=heatTransfer.useAlphaAState
        "sets alphaA as differential state"                                                                       annotation(Dialog(group="Parameters", tab="Advanced"));
      parameter Real falseDynamicsTimeConstant=0.1
        "Time constant of false dynamics for pressureDropState" annotation(Dialog(group="Parameters", tab="Advanced"));

      parameter Real alphaAStateTimeConstant=1 "Time constant for alphaAState"
                                        annotation(Dialog(group="Parameters", tab="Advanced"));

      parameter TIL.Cells.Internals.VLEFluidCellOrientationType orientation="A"
        "Selection of cell orientation for equations of mDotHydraulic and pressure" annotation(Dialog(group="Parameters", tab="Advanced"));

      parameter TIL.Internals.CellFlowType cellFlowType = "allow reverse flow";
      inner input TIL.Cells.Geometry.FluidCellGeometry cellGeometry;

     /****************** Connectors *******************/

      TIL.Connectors.VLEFluidPort portA(final vleFluidType = vleFluidType,
        m_flow(final start=m_flowStart)) "Fluid port A"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=
               0)));
      TIL.Connectors.VLEFluidPort portB(final vleFluidType = vleFluidType,
        m_flow(final start=-m_flowStart)) "Fluid port B"
        annotation (Placement(transformation(extent={{90,-10},{110,10}}, rotation=0)));
      TIL.Connectors.HeatPort heatPort "Heat port"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}}, rotation=
               0)));

     /****************** Initial and Start values *******************/

      parameter SI.MassFlowRate m_flowStart "Start value for mass flow rate";
      parameter SI.AbsolutePressure pStart "Start value for pressure";
      parameter SI.SpecificEnthalpy hStart = 0 "Start value for specific enthalpy";
      parameter SI.MassFraction[vleFluidType.nc-1] xiStart = zeros(vleFluidType.nc-1)
        "Start value for mass fraction";
      parameter Real alphaAInitial = 0 "Initial value for alphaA";
      parameter SI.Pressure pressureDropInitial = 0
        "Initial value for pressure drop";

     /****************** VLEFluid models *******************/

      final parameter Boolean computeTransportProperties = heatTransfer.computeTransportProperties or pressureDrop.computeTransportProperties;

      TILMedia.VLEFluid_ph    vleFluid(
        final computeTransportProperties = computeTransportProperties,
        final p=p,
        final h=h,
        final xi=xi,
        final vleFluidType=vleFluidType,
        computeVLETransportProperties=computeTransportProperties) "VLEFluid model"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));

     /****************** Properties of control volumes *******************/

      SI.Mass mass "Mass of vleFluid in cell";
      input Real dpdt "Derivative of pressure wrt time";
      parameter Boolean steadyStateContinuity = false;

     /****************** Heat transfer and pressure drop *******************/

      replaceable model HeatTransferModel =
          TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer
        constrainedby TIL.Cells.TransportPhenomena.PartialFluidHeatTransfer
        "VLEFluid heat transfer model";
      HeatTransferModel heatTransfer "Heat transfer"
        annotation (Placement(transformation(extent={{-10,-50},{10,-30}}, rotation=
                0)));

      replaceable model PressureDropModel =
          TIL.Cells.TransportPhenomena.PartialPressureDrop
        constrainedby TIL.Cells.TransportPhenomena.PartialPressureDrop
        "VLEFluid pressure drop model";
      PressureDropModel pressureDrop "Pressure drop"
        annotation (Placement(transformation(extent={{-10,30},{10,50}}, rotation=0)));

      SI.Pressure pressureDropState(final start=pressureDropInitial, fixed = fixedPressureDropInitial)
        "Pressure drop used in false dynamics model";
      Real alphaAState(final start=alphaAInitial, fixed=useAlphaAState)
        "AlphaA used as differential state";
      parameter Boolean fixedPressureDropInitial = false
        "if true, force usage of initial value";

      Real drhodt "Time derivative of density";
      inner SI.MassFlowRate mdotHydraulic(
        final start=m_flowStart/cellGeometry.nParallelHydraulicFlows)
        "Hydraulic mass flow rate";
      inner SI.HeatFlowRate QdotHydraulic(start=0) "Hydraulic heat flow rate";
      inner SI.Temperature wallTemperature "Wall temperature";
      SI.TemperatureDifference delta_T "Temperature difference";

     /****************** Additional inner and outer objects *******************/

      inner TILMedia.Internals.PropertyRecord properties = TILMedia.Internals.PropertyRecord(
        d=vleFluid.d,
        h=vleFluid.h,
        p=vleFluid.p,
        s=vleFluid.s,
        T=vleFluid.T,
        cp=vleFluid.cp,
        q=vleFluid.q,
        VLE=TILMedia.Internals.VLERecordSimple(d_l=vleFluid.VLE.d_l, h_l=vleFluid.VLE.h_l, p_l=vleFluid.VLE.p_l, s_l=vleFluid.VLE.s_l, T_l=vleFluid.VLE.T_l, d_v=vleFluid.VLE.d_v, h_v=vleFluid.VLE.h_v, p_v=vleFluid.VLE.p_v, s_v=vleFluid.VLE.s_v, T_v=vleFluid.VLE.T_v),
        VLETransp=vleFluid.VLETransp,
        transp=vleFluid.transp,
        crit=vleFluid.crit);

       parameter Boolean removeSingularity annotation(Evaluate=true);
       parameter Boolean generateEventsAtFlowReversal  annotation(Evaluate=true);

      parameter Real yLimit=0.9;

      SI.AbsolutePressure p(final start=pStart);
      SI.SpecificEnthalpy h(final start=hStart);
      SI.MassFraction[vleFluidType.nc-1] xi(final start=xiStart);
    equation
      if orientation =="A" then
        p = portA.p;
        mdotHydraulic = portA.m_flow/cellGeometry.nParallelHydraulicFlows;
        if steadyStateContinuity then
          portA.m_flow + portB.m_flow = 0;
        else
          drhodt*cellGeometry.volume = portA.m_flow + portB.m_flow "Mass balance";
        end if "Mass balance";

      elseif orientation == "B" then
        p = portB.p;
        mdotHydraulic = - portB.m_flow/cellGeometry.nParallelHydraulicFlows;
        if steadyStateContinuity then
          portA.m_flow + portB.m_flow = 0;
        else
          drhodt*cellGeometry.volume = portA.m_flow + portB.m_flow "Mass balance";
        end if "Mass balance";
      else
        p = (portA.p+portB.p)/2;
        mdotHydraulic = (portA.m_flow/cellGeometry.nParallelHydraulicFlows - portB.m_flow/cellGeometry.nParallelHydraulicFlows)/2;
        if steadyStateContinuity then
          portA.m_flow + portB.m_flow = 0;
        else
          drhodt*cellGeometry.volume = portA.m_flow + portB.m_flow "Mass balance";
        end if "Mass balance";
      end if;

    // Set false dynamics model
      if (useFalseDynamics) then
        portA.p - portB.p = pressureDropState;
        der(pressureDropState) = (pressureDrop.pressureDrop - pressureDropState)/falseDynamicsTimeConstant;
      else
        pressureDropState = pressureDrop.pressureDrop;
        portA.p - portB.p = pressureDrop.pressureDrop;
      end if;
      if (useAlphaAState) then
        heatPort.Q_flow = alphaAState*(heatPort.T - vleFluid.T);
        der(alphaAState) = (heatTransfer.alphaA - alphaAState)/alphaAStateTimeConstant;
      else
        heatPort.Q_flow = heatTransfer.alphaA*(heatPort.T - vleFluid.T);
        alphaAState = heatTransfer.alphaA;
      end if;

      portA.h_limit = h + yLimit*vleFluid.d/vleFluid.drhodh_pxi;
      portB.h_limit = h + yLimit*vleFluid.d/vleFluid.drhodh_pxi;

      if (removeSingularity) then
        portA.h_outflow = noEvent(max(h,inStream(portA.h_limit)));
        portB.h_outflow = noEvent(max(h,inStream(portB.h_limit)));
      else
        portA.h_outflow - vleFluid.h = 0;
        portB.h_outflow - vleFluid.h = 0;
      end if;
      portA.xi_outflow = xi;
      portB.xi_outflow = xi;

      drhodt = vleFluid.drhodh_pxi*der(h) + vleFluid.drhodp_hxi*dpdt + vleFluid.drhodxi_ph*der(xi);

      QdotHydraulic = heatPort.Q_flow/cellGeometry.nParallelHydraulicFlows;
      wallTemperature = heatPort.T;
      delta_T = heatPort.T - vleFluid.T;

      mass = cellGeometry.volume*vleFluid.d "Mass in cv";

      if (generateEventsAtFlowReversal) then
        der(h) = 1/mass*(portA.m_flow*(actualStream(portA.h_outflow)-h) +
          portB.m_flow*(actualStream(portB.h_outflow)-h) + heatPort.Q_flow + cellGeometry.volume*dpdt)
          "Energy balance";
        der(xi) = 1/mass*(portA.m_flow*(actualStream(portA.xi_outflow)-xi) +
          portB.m_flow*(actualStream(portB.xi_outflow)-xi)) "Mass balance";
      else
        der(h) = 1/mass*noEvent(portA.m_flow*(actualStream(portA.h_outflow)-h) +
          portB.m_flow*(actualStream(portB.h_outflow)-h) + heatPort.Q_flow + cellGeometry.volume*dpdt)
          "Energy balance";
        der(xi) = 1/mass*noEvent(portA.m_flow*(actualStream(portA.xi_outflow)-xi) +
          portB.m_flow*(actualStream(portB.xi_outflow)-xi)) "Mass balance";
      end if;

      annotation (Icon(graphics={Bitmap(
              extent={{-100,-100},{100,100}},
              imageSource=
                  "iVBORw0KGgoAAAANSUhEUgAAAMgAAADICAIAAAAiOjnJAAAACXBIWXMAAAsTAAALEwEAmpwYAAABhElEQVR42u3UsQ3AMAhFQZPJ2JzRSEvhynKkFHcd7dcTawEAcFvMo6oswpnMjIh9WN1tIM6/1AjrMQdfEBbCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAuEhbAQFggLYSEsEBbCQlggLISFsEBYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAuEhbAQFggLYSEsEBbCQlggLISFsEBYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYSEsEyAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLAAD4qReb+Qd/FUGUbAAAAABJRU5ErkJggg==",
              fileName="modelica://TIL/Images/CellUni.png"),
                                       Text(
              extent={{-80,40},{80,-40}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="VLEFluid")}),
                            Diagram(graphics));
    end VLEFluidCell;

    model WallCell "Wall cell model"

     /****************** General parameters *******************/

      inner input TIL.Cells.Geometry.WallCellGeometry cellGeometry;

     /****************** Connectors *******************/

      TIL.Connectors.HeatPort portN(T(start=TInitialWall)) "Heat port north"
        annotation (Placement(transformation(extent={{-10,90},{10,110}}, rotation=0)));
      TIL.Connectors.HeatPort portS(T(start=TInitialWall)) "Heat port south"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}}, rotation=
               0)));
      TIL.Connectors.HeatPort portW(T(start=TInitialWall)) "Heat port west"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=
               0)));
      TIL.Connectors.HeatPort portE(T(start=TInitialWall)) "Heat port east"
        annotation (Placement(transformation(extent={{90,-10},{110,10}}, rotation=0)));

     /****************** Material *******************/

      replaceable model WallMaterial = TILMedia.SolidTypes.TILMedia_Aluminum
        constrainedby TILMedia.SolidTypes.BaseSolid "Wall material" annotation (choicesAllMatching=true);

      SI.Temperature wallStateTemperature;
      TILMedia.Solid wallMaterial(
        T(final start=TInitialWall)=wallStateTemperature, redeclare model
        SolidType =
            WallMaterial) "Wall material in center of cell"                                  annotation (Placement(
            transformation(extent={{-40,20},{-20,40}}, rotation=0)));

      replaceable model HeatTransferModel =
          TIL.Cells.TransportPhenomena.PartialSolidHeatTransfer
        constrainedby TIL.Cells.TransportPhenomena.PartialSolidHeatTransfer
        "Heat transfer model";
      HeatTransferModel heatTransfer "Heat transfer"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));

     /****************** Additional variables *******************/

      parameter SI.Temperature TInitialWall "Initial wall temperature";
      parameter Boolean fixedTInitialWall=true
        "if true, force usage of initial value TInitialWall"
        annotation(Dialog(group="Wall", tab="Advanced Initialization"));

      SI.Mass geometricMass = cellGeometry.volume*wallMaterial.d "Geometric mass";
      TIL.Utilities.Units.EntropyFlowRate SProd_dot=
                                          -(portN.Q_flow/portN.T+portS.Q_flow/portS.T+
        portW.Q_flow/portW.T+portE.Q_flow/portE.T) "Produced entropy";

     /****************** Additional inner objects *******************/

      inner TILMedia.Internals.SolidPropertyRecord properties(
        final d=wallMaterial.d,
        final T=wallMaterial.T,
        final cp=wallMaterial.cp,
        final lambda=wallMaterial.lambda) "Property record";

      parameter TIL.Cells.Internals.WallCellStateType wallCellStateType=
                                            "state center";

      parameter Boolean initSteadyState=false;

    initial equation

      if initSteadyState then
        der(wallMaterial.T) = 0;
      elseif fixedTInitialWall then
        wallMaterial.T = TInitialWall;
      end if;

    equation
      portW.Q_flow = (portW.T - wallMaterial.T)/heatTransfer.R_WE*2.0;
      portE.Q_flow = (portE.T - wallMaterial.T)/heatTransfer.R_WE*2.0;
      geometricMass*wallMaterial.cp*der(wallMaterial.T) = portN.Q_flow + portW.Q_flow + portE.Q_flow + portS.Q_flow;

    if wallCellStateType == "state north" then
      wallMaterial.T = portN.T;
      portS.Q_flow = (portS.T - wallMaterial.T)/heatTransfer.R_NS;

    elseif wallCellStateType == "state south" then
      wallMaterial.T = portS.T;
      portN.Q_flow = (portN.T - wallMaterial.T)/heatTransfer.R_NS;

    elseif wallCellStateType == "no wall resistance" then
      wallMaterial.T = portS.T;
      portN.T = portS.T;

    else
      portN.Q_flow = (portN.T - wallMaterial.T)/heatTransfer.R_NS*2.0;
      portS.Q_flow = (portS.T - wallMaterial.T)/heatTransfer.R_NS*2.0;

    end if;

      annotation (Diagram(graphics),
                           Icon(graphics={Bitmap(
              extent={{-100,-100},{100,100}},
              imageSource=
                  "iVBORw0KGgoAAAANSUhEUgAAAMgAAADICAIAAAAiOjnJAAAACXBIWXMAAAsTAAALEwEAmpwYAAABhElEQVR42u3UsQ3AMAhFQZPJ2JzRSEvhynKkFHcd7dcTawEAcFvMo6oswpnMjIh9WN1tIM6/1AjrMQdfEBbCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAuEhbAQFggLYSEsEBbCQlggLISFsEBYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYYGwEBbCAmEhLIQFwkJYCAuEhbAQFggLYSEsEBbCQlggLISFsEBYCAthgbAQFsICYSEshAXCQlgIC4SFsBAWCAthISwQFsJCWCAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLhIWwEBYIC2EhLBAWwkJYICyEhbBAWAgLYSEsEyAshIWwQFgIC2GBsBAWwgJhISyEBcJCWAgLAAD4qReb+Qd/FUGUbAAAAABJRU5ErkJggg==",
              fileName="modelica://TIL/Images/CellUni.png"),
                                       Text(
              extent={{-78,42},{82,-38}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString=
                   "Wall")}));
    end WallCell;

    package Geometry
      extends TIL.Internals.ClassTypes.ModelPackage;

      record FluidCellGeometry
        extends TIL.Internals.ClassTypes.Record;

        Modelica.SIunits.Length length = 0;
        Modelica.SIunits.Volume volume = 0;
        Modelica.SIunits.Area heatTransferArea = 0;
        parameter Real finHeatTransferAreaRatio = 0
          "Fraction of fins on fin side heat transfer area";
        parameter Modelica.SIunits.Area hydraulicCrossSectionalArea = 0;
        parameter Integer nParallelHydraulicFlows(min=1) "Number of parallel flows";

      end FluidCellGeometry;

      record WallCellGeometry
        extends TIL.Internals.ClassTypes.Record;

        Modelica.SIunits.Length length;
        Modelica.SIunits.Volume volume;

      end WallCellGeometry;
    end Geometry;

    package TransportModules
      extends TIL.Internals.ClassTypes.ModelPackage;

      model SolidHeatTransferModule "Heat transfer model for solids"
        SI.ThermalResistance R_WE "Thermal resistance (W-E) for one cell";
        SI.ThermalResistance R_NS "Thermal resistance (N-S) for one cell";

        inner input TILMedia.Internals.SolidPropertyRecord properties
          "Material property record";
        inner input TIL.Cells.Geometry.WallCellGeometry cellGeometry
          "cell geometry record";

        replaceable TransportPhenomena.PartialSolidHeatTransfer heatTransfer
                                                            annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

      equation
        R_WE = heatTransfer.R_WE;
        R_NS = heatTransfer.R_NS;

        annotation (Icon(graphics={
                               Bitmap(extent={{-100,-100},{100,100}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/LambdaGeneral.png")}));
      end SolidHeatTransferModule;
    end TransportModules;

    package TransportPhenomena
      extends TIL.Internals.ClassTypes.ModelPackage;

      partial model PartialFinEfficiency "Base class for fin efficiencies"

        Real eta "Fin efficiency";

      protected
        outer SI.CoefficientOfHeatTransfer alpha "Coefficient of heat transfer";
        outer TILMedia.Solid finMaterial annotation (Placement(transformation(extent=
                  {{-10,-92},{10,-72}}, rotation=0)));
        outer TILMedia.Internals.PropertyRecord properties "Property record";

        annotation (Icon(graphics={Bitmap(
                extent={{-100,-100},{100,100}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/FinEfficiency.png")}));

      end PartialFinEfficiency;

      partial model PartialFluidHeatTransfer
        "Base class for heat transfer in fluid cells"

        SI.ThermalConductance alphaA "Value for alphaA for one cell";
        parameter Boolean computeTransportProperties = true;
        parameter Boolean useAlphaAState = false "sets alphaA as differential state";

      protected
        outer SI.MassFlowRate mdotHydraulic "Hydraulic mass flow rate";
        outer SI.HeatFlowRate QdotHydraulic "Hydraulic heat flow rate";

        outer SI.Temperature wallTemperature "Wall temperature";

        outer TILMedia.Internals.PropertyRecord properties "Property record";

        outer TIL.Cells.Geometry.FluidCellGeometry cellGeometry
          "cell geometry record";

        annotation (Icon(graphics={
                               Bitmap(extent={{-100,-100},{100,100}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/Alpha.png")}));

      end PartialFluidHeatTransfer;

      partial model PartialPressureDrop "Base model for pressure drop in cells"

        SI.Pressure pressureDrop "Pressure drop for one cell";
        constant Boolean computeTransportProperties = false;

      protected
        outer SI.MassFlowRate mdotHydraulic "Hydraulic mass flow rate";

        outer TILMedia.Internals.PropertyRecord properties "Property record";

        outer TIL.Cells.Geometry.FluidCellGeometry cellGeometry
          "cell geometry record";

        annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}},
                imageSource=
                    "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",
                fileName="modelica://TIL/Images/DeltapIcon.png")}));

      end PartialPressureDrop;

      partial model PartialSolidHeatTransfer
        "Base class for heat transfer in solid cells"
        SI.ThermalResistance R_WE "Thermal resistance (W-E) for one cell";
        SI.ThermalResistance R_NS "Thermal resistance (N-S) for one cell";

      protected
        outer TILMedia.Internals.SolidPropertyRecord properties
          "Material property record";
        outer TIL.Cells.Geometry.WallCellGeometry cellGeometry "cell geometry record";

        annotation (Icon(graphics={
                               Bitmap(extent={{-100,-100},{100,100}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAMgAAADICAYAAACtWK6eAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAFkVJREFUeNrsXetSG0mazRIgwMhItrt7entmA3ljYiI2YiZQx/o/mieAeQLTT7DMEyz7BvgJmn4D8QTI/9mwiJ2dHxsTg9junnZfbBB3BEJbn/gKl0FVlVWV+VVm1XciMuQL6FLKk+ecLy/lCIZy7AhRdR8abqtjq+Hfxb0/x0XHbYf3/tzF1nkhRI+vvlo4fAmUEKHhI0Qz47fV9giDpHnN3xQThIoQC0iApo8UNqCDDcjTdkmzz98mE0SVQgAZVvCxnpOP1kWytJAwbM2YILFUYsVHjCKg5RGG1YUJEqQUq9gaBb8cYMU2kCw9JkixibGMpNCmFDNum3BbGZv/3+Jg4LYL/HMfm//fNCrLpkuULSZIsSyUpxbKMsUcdvyy77FM9Jk8wlz4Hk/VZ5ZNJMs+EySfxFh0H9aQGKkwhYTwSDFj6Gf2iOI9Xql5WiDKhkuUXSZIPoix5D6sixTzEyUkw2N8nLL0WlwhUY7x8Sbd00GoX8/7PIvDxAhWiYqPGHmER5STdOqSa6I4TIyPlQJIASWtRwXLZmfitmR1klxZckkUJ0fEWEBixM4Ys26bR2Iwboly5Lbz5BllPS9h3skBMaoYvtfjqgWQomZxpqDILIdIlgSqso5hvscEyY4cMI8Bk1p12d+ZdNtTzBYTzAEpDDCrvHfbdbxf7cLgZfM8imMpMRaQGCtxiPEkx4GbMtgfxCdKC4myzwTRT45/R/muyRKjxsTQQpTDeEQ5xGzyigmiL2vASNSUzRhgpSrcl7XiBK1XjIzSBuW3JZs4lpADssamrGpUUTFK3H9JcIOKEqPHg5qs2pBNHMOJUUU7tSbz89OoGpPcZzPBNarJpfyvbKDt6jFBkgVxsFSRy88nUFpmuY8agXOUiIHcj3fQcu0zQTRYKijXzgve2GIahuJ2/kRyVbGxlssxkBz/ISQm/TzVKHNfNBr9eGoCdus/mSDB5PhaSCwVmcYgziHcnhDfk88msOfkKybIwzDejsob8GahOjXDfc5KwL6UY7RfErmkaUJ4dwwgh1QYn8CswRUqu3GN2WQgR5LMw7uTMTkWUTlCwzjkjApbqlxZrhPMJxLhvZnl7kXHdHJA6fYR96lcAvagnBtOEsdUcjhIDK5S5Rt9JMrQUJI4ppKjIng5elEwQMtlIkkcE8nBezWKSZJTA0nimESOEtoq02Yvf7O9rfX5Lzsd8fOf/1x4kgzRbt0YRBKHiBxQyu1EkWNWmLlk5POvvxbV1VWtr/H3el1c7fOxuEMM7hIkaVCUgEsE5PD2cYSSY8Z3gUxrx62W9o4x7xJwaOjnp2wC+0JEx4S+1NohOGfDISDIGxEyCegIO2bGf7u3J8ruKK+tmtPtir89f85hxIeL6EwCNwj60loFwbVVoeQoWzKyHW5uau0MQL7K8jKriK+Vo0fwBvYx+wiCq3JDjfuUwbbqfjvQTJCRb2Cb9cBuSRzJtIp9zR6Lhfs5Qo37pLBvD8e/bG+LSrOp9TX+p1YTgx7f8Ol+cJc4HGJFx36SkgZyQMUqdLidsEg5/O09q0hmSiIxL7aJfc9cBZFZtu4Iuxcd/uHgQEzUatqeH8L6Xzmsj8WNRGgXipfJq+6r60JiD/mNxe1Qc8kXwvrc0pLV10hXk0BDxDyClowgmDsiTx+xXe5/3NjQPlI+ZZsVarcisIZ90RyLhdaqKyTPrbId//rmjXjU0Huvzw6H9TSAmfa6CqulSkFaRSEH4B1BWH+meWlLzlETEVVUMgXBs3I3inT1J6pV0Tg81Poal25Y/wuH9bRYS3sWcCklORZUhyIbANbnF80qMu2G9dnFRe7iKYtGaUu/aS3WRpGs1Ucml2AB42dra9zF01utVO4mscWSmS3PO36/tzca6XXh2rVxf3Gfn8N6aiSeZS8lJEe1aLkji7A+WauJ2soKd+/02Ei6ND6pxQLtrzNBNrW/BtssJagLyTsEpLZYGHq6fM1v8bvtbfFY8wLGvzYa4nx3ly+2AqLE3YWYREE2+DqziliKda0K4qrHkrhdjMjwYfHgYJQXOKxbAVjM+FqXgqzz9X0I3SVfDuvZqYg0QVA9mnx9H+InggWMvPREnYJgX1auIKweAYAAfdbpaH0NKASUFxb4YhOriBRBWD04rBdVRRxJgmwzQcJBsYARwvrukyd8sdWg7Yb1P6ZWEDwylMkRAagwUYT1Zy9f8sVWpyKLqQkiEs5AFhG/8D4R2xDZt50I9eBZ85jQvYAR8N/u8/f5HF9VCJ1dj1IQHq5igmIZPKuIUqymUZA9wYsSYwFKsX/o6hVd3m2oFF1XQZ7HVhDc78HkiAmwPsftttbXAAtXXV7mi63IYoWdglJie6UeFHMin7DNIrFZToB6wOaSQ75uyQBzIr93bZDOBYwc1pWjNu6YoBKrh3pQzIlwWFeOlTgWi6+8gpzABLEKa1IEwbmPBl+v5IBKlu5dhhzWlaMx7oigkqzUMMwc2XmfiF6bNY4gTb5O9hAEqllQFGAoQTOUIFi94iEpBcDyUOQPziJ6FOT+8UAlVg/7LQ/vE9GnIiXOH+oAVieLCTxQrMrSEn8BGnIIK0hOAjPbLD0K4vjyBy9tTwmKG+sEgY8GUoq7JfAlVg81gFsVZEUOAB8NpEdFmCA5sjgc1vUShGfPLScIKBjfdEcJGkwQleR4+VL7yl1WkQwJEuekOcZDmOT9OYeogceJEqtHOsDCRJM6JR8NpFZFmCA5yB42vCfbCVLn65Gfzsjn+CpB3U+QJl+P+IDlHdQLEzmsk2HEiVLSmxsyzLYybLOUBPVqifNHMmS1MDFOWOfdhulzSInzRzLYUE7lo4HS5xAmSI49PpCYw3p6gtT4OsRD1gsTOYvQjTGcQXLe6Zgg6TMII8edjo8GSge2WDEBnc2UhYkc1tlicWdTFNb5aCC2WNph2sJEziI0FosRYyS2Fbz0hAnCnSwirPPRQEwQbdC9MBFuq3bW6bDNYoLYCd2dC+4novvOVHyOLxNEC6BT6c4fQA6Km+7wllwmiJZOpXPuA+zV+e7u6HZqum0Wh3WFBHG4jZruTnXSbt+91onmO+TCGrJHi4v8vfoaK0gKwNyH7oWJfmtFcYdcVhEFCsIjC416wJm6R1tbd6934VotsFw6UQXL6OYq/n6jVYQVJAJPNVevemOCeU9zWB/tNuSwzgqSttUIFiYeuWS4/7oHBDYLytb8HbOCGK0eA9de9Vx7dR+jipZmm1Xho4GkFaTDKvKwQefRbUN6Y9TDaz2COZFP3XzF6hGKDhDkkMeJ8UFWN45CSEBhs57y0pPIAmOgxRoWfFT5VHP1anCvenW/XRDYrAk3Xz19+bLwCjJMarGKirmlJVHWfGLikYSFOiKwWfNczUpusYo6olBYj6OQ/EFZzari0UCcQYItVjfIYhURsDBR96jq2asogM067+gX+CcFzyIhfb0bSJCiqgiQY0Lz3Acog+z7OSRQkScFnhOJQDc0gxRRRShG0zid/pggh0Demi/o0UDDqAzyQogeV7I+zH3MNZtavxCoTF261kn2PV3t75OE9VpBVSSMIMANr8zbZgUR4hnBKtf3Gxuxf+eYqJpVxN2GIX18xAmPIF1WECEeE5Q8jyWqVw9m1b/5ZhTsWUVIFaTrJ8jYHHJToJHksevBdc99nLbbI8uUBD2KmfUC7hMJ6eOdSIIUSUVqBOG8F6N6lUU1CwYImCRl9YhBkCKoCHjvxwRzHycpsgQE+wuCOZFqgeZEbsID+us7gmAlK9Bm5X0koegUQI6bXi/V+6SwWTU8GqgIChJlr/wKEqgigwJcKArvncZeee2IgCBFCuuDmAQJLPXm2WZNLy6KKc3h/AqO9Xn9Or0lcBWIQkXmC2CzbiRKvFIEybvNekKgHkcK1OPuWCCCOZGZRkPM5PxooIhB/yFB3ByyLwLmQ65zOoqUXK9dodgYpXDUP93aGimSdpuV85JvSJ/uIhceKEigilzndBSpECxMPGu3xfX+vtL3TaEiFZxZz6uCXEuoxziCBF75qxyOIhRe+1hDZjhMsFwldunbHTgqOd1MFdGXP+LARyt+d4SAxThj1zSU3Tabo4s0ubAg6gRW5e9uR4NwrRq/3t4Ws5oXVl52OuLbL7/MHUHO3dYPcZf+BbwfKQj+R6sIClIjCuc6yKE61wRW+NywPpnDo4FC+nLr/ur2UpQH8wAlsTzNicwR2IfTBAsTZduZ+9wkCxhzdjTQQMiVd8MIEphD+jkZQeaWl0nmPs4kttUmBSjTGUFYf5yzOZF+jPwxliBY4uoESVMulrUTfOlnGtXjbukJUVh/nKOjgULsVcdf3g1TEMBmkM2yXUVg7oPCXlF03v7uLsmcSF5UpB9ur8Z+YbEIMqpssHpIVX8Giuc+AtdnERARqmVTOTga6DKmvQokSFg1CyZYbF6bVaE484qg0/qtXF6um07ciNDJwVbQ2Qxhp7sHqsiFpSMILEyc1ny3KMA5Qf64q8rAfQ2JwrrN6nER/vEC+3ogQVxGQQmmG2SzbDzQYZ5g7uNE49xH2GvqxmS9Lh5ZejTQMNxedbGvxyOITFi3bRR5RBDOzwjVw2vnW1vihmBOpGKpikQM6KGjSyKCjDqCZRep8vKlKGlemAid9CLkxHadjUJFYICZtDCsn4d/rI3EBMG68GZQ6LmwSGbnKLbVEu34G4dTotd+ZNkCxovwotJm2MGJQkQcT7pz+7AoAiYOgV2fWHCRJtxR7wuC+YK3jYa42t3N7HP+6s0bUdZchLh2r+MPz59bQ5BfwgkCF2v3RQqLBYBvvB2kIuesHiP0O51MyQE4JigvQ1ifXlqyghzn4eRoY98WIiVBAOuB0m6BB6UgyJnCbbWJS5lEcyJzloT104R9WoogOx//9bWtKjIDt3LWvDDRI0jm5cxejySLjAhi+Dm+EurxOqCvJ1KQUMYdC3NPYJwlCJUwMThMeeaVsmXwREQ1WUWG2CfTqkdgSA9h1LbbmuP+o4LNJMAo9wXB/MA7l4QXGpe2xw7re3vaVRPC+o+GhvUTbCHq8cdx//EipYKEMg/ekGkbqiiyB8x9XGY09xFY9yeaWZ8y8GigQTg5YqlHEoKAbwu8+j3DRpJHROHcNFC9p4qBRwNF9MFNf/ZIZLF2on8HNil3g/7ziTDjcIdJd3T7lOCw558bDXGdcXl3HJ5ub4tpzYc6gHr+5CrJsGfG0AjB/CD8R8B3ht5/4kVKBRH4AoEydSTMWA4/RzC6DVwfbiI5BJHNgqU7M4bMrN9g34uwVrFvzpJEQQBQ4+sgIx92TrfVMr5gnx0caF97deSS8OzVK2EqKK7BlavS7ww4GghKMSHzHuB4GjIpIFRBduTfD7xQ4BANb/Qyw4s1S7AwEXBJNDFn8vubgqOBFhez/ZwiclJwTTYi7yiwWB6grhn4DYAXzGpuZJpA9mHW+oZoW23iORGinY2zGc6JDKNzR0uE7PeItJEpr82aCDiJEcpt7zMYTWBhIoUv7huuHgDIRwOCRZqzGW7HfS9C7/NxGOZ0KAgSGthhqTH1vpFZormPi2++sWI9EoWKgJ2dzeBooDMRueUiUTAfS5Cd5M8BKbUdZrUojy2doVi5a4F63PlzojmRGWIVuYq2Vm3sm7Gxo1BBPKwEWS3wiO+I8kh5aUlMUCxMdEdla+7k2uuRhPVyszmyt1S5w+tTIdZKic9WRRCoEKyGsf0dwYWjGMXA09+43t6qPdlUuw2JzvF9F+1KVoWihR0lhdcHKgWBhvc8SGIUARYmUhDknPDMK2WWkOhQhzJBceRQSO0xV7ZydFLx+4dQ1BS3kzJjZWZK6Fn1C0HxbH1dv/fFU0tsw4k7eEwSnAkGNgvK31o+Q7QsdETMxYiRA2/KgD4OC/hGa0GS9bm4vSEPgyGtgm77R7S4NNJWrfx4oUFBBL5B8DpjkyGsmXmLJJnm750hgUvsMyI6dyiXrpKmz7QVJnVAkp9Evu+/zlADyb6yrjJ36LZYfnwtQqpbYLP+CXwr9wPGGMAM+Q8i8pYbm277Ssfrv9CoIB6+EgFnanm+8gdWEkaAckiQo6OLHHcKsqP/s8LS+LYIqGx5SvIFKwnDpxz/kCNHU2jeyFoi+LzwAQJn2v0VClYSxo0cObyZcu1bGSeJPvc+sh2UpBZEku/d9mtWkkIrx/dy5GgKDRWrrBTEwy5+sFAl+VZku9mKkQ0uscdLkoNsn/Mk8XXYjVKSaxxFfiN4nqRI5Pg+2mKTk4NaQaSVBC7UdyJyEz4jB4Dv+P8MJUcWCiKtJHDBfkRFecb9KJd4h00YSo6sFERaSbyLKCG/DItwg9+p6eTImiAeSWB+JPSEt1NfeHe4WX+v8m9F5CkkAvtEI0tymEAQIT6UgDtRQQ4ubI8HYGvRE9JVSm8ScD/r91wy6NrB6WObUdL81me5eES2o3mW6q2cVd7EvmDEWFgybJCBdTXrUT8EG2e64vZUC4bZOMPv6kTux9eF5rVVcUGxFisJlnEkiTweEQ7L/lTw7LtpgFnxn0XkySP+ML4qNC1ZzyNBALAzsSVCFjl6gG28v3LbPPdLIwBzG1CilzzuCfLGigl5wwaLNS68R56SAF/Edyjlfe6fmaGP38F38uTYMCWM26ggiSyXQMv1jG0XqZ16h5ZKEsZaqlgEMen0juHtvpKWCLhH4jhp/BwzCkMfIGNIVqc8tMFSOQZV7IeWWqz7ZO05tzdfDDww2w9vX8H/4g9zuVVtO8RrG2Mfz+ggafgOHYumswIVxDFMQe51/gX0r9InlUGQ/4wVRYli/CRin7cMyr9WMjRrDENUxEqC+IiyjESpxyGKV/HijCKfMWJWpjx0kRhGZ41EBClZQBD88qpou9bj/F4J1eQTwYfYBQGqUr+gaiRYLArfx8aEBXZqGPL5rCeIjygL+KWsxv3dOSTLU+bECO+RFKfJfn0TvocJg0u3yghSsvDLvRZiSXw4IzgW4PNWkSyVgpHiBEnRE4m3FrThuk/GvA+5IVa9OARRQRQvq1SRKNWckqInPhwIneImR9YSIxVBwF5N5qATXKUkijdYVHyEsTWz9H2EOBGpN6GNiDFlMTF8g2myKlYeCOIjyiKG+dW0z1VGosz6Hk3EORLBe1S0DAcyxsZUxhuZjCDIVA5tRf82zK9iq6t6Xo8oZd9jme4zjdq57/FE7Ut0kRibZYvCd4zBMz5BhMh/+bN/O48CRNF2ayQgy8Q9wnj/FgcD8eHOSh4h/P+mCS0kxVbO+0EgIgniiPzj8jZerKAFa4hiA5afw+Rra7oAO5yHaQgyJeyuZCUkywKSpalTWQxDC0M3kGK/SN/3jQiv4IXeH2QyZ0E9ClMLC+J33e7d3//Lcao+ojRVZpaM0fUIAY//Nhz2jlot8e2f/lQ4ubzGNg6Rt2AbFuxiPVtb++jv0HHErf/eQsIsIFGaaMVssWMdbG0kxAOVmF9ZERPVqhj0inVuTFQfD1UQsFdFOh/3t3t7olyPJxIuaZZ8ZKmLFPMtitBGhRiRwiWD9DzFW3eAeP/qVdEsdeB80AshcQu22YJcqMfLy+KfWy0lz4XWzCMMtJpPbWoplAc6/eG9P3c9QqDiJUbftZd/e/68UAQJqwICQf5fgAEAMqGfBZAZKC8AAAAASUVORK5CYII=",
                fileName="modelica://TIL/Images/Lambda.png")}));

      end PartialSolidHeatTransfer;
    end TransportPhenomena;

    package Internals
      extends TIL.Internals.ClassTypes.InternalPackage;

      type WallCellStateType
        extends String;
         annotation(choices(
          choice="state center"
              "differential state T in center of cell - default option",
          choice="state north" "differential state T in north port",
          choice="state south" "differential state T in south port",
          choice="no wall resistance" "no wall resistance between north and south"));
      end WallCellStateType;

      type VLEFluidCellOrientationType
        extends String;
         annotation(choices(
          choice="A" "mDotHydraulic and p result from portA",
          choice="B" "mDotHydraulic and p result from portB",
          choice="symmetric" "mDotHydraulic and p result from portA and portB"));
      end VLEFluidCellOrientationType;
    end Internals;
    annotation(classOrder={"*","Geometry","TransportPhenomena","Internals"});
  end Cells;

  package Connectors "Connectors used in the component models"
    extends TIL.Internals.ClassTypes.ModelPackage;

    connector GasPort "Gas port"
      parameter TILMedia.GasTypes.BaseGas           gasType;

      SI.AbsolutePressure p "Pressure";
      flow SI.MassFlowRate m_flow "Mass flow rate";

      stream SI.SpecificEnthalpy h_outflow
        "Specific thermodynamic enthalpy close to the connection point if m_flow < 0";
      stream SI.MassFraction xi_outflow[gasType.nc-1]
        "Independent mixture mass fractions m_i/m in the connection point";

      annotation (Icon(graphics={Ellipse(
              extent={{-80,80},{80,-80}},
              lineColor={255,153,0},
              lineThickness=0.5,
              fillColor={255,153,0},
              fillPattern=FillPattern.Solid)}),
                                Diagram(graphics={Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={255,153,0},
              fillColor={255,153,0},
              fillPattern=FillPattern.Solid), Text(
              extent={{-100,100},{100,60}},
              lineColor={255,153,0},
              fillColor={255,153,0},
              fillPattern=FillPattern.Solid,
              textString=
                   "%name")}));
    end GasPort;

    connector VLEFluidPort "Fluid port"
      parameter TILMedia.VLEFluidTypes.BaseVLEFluid           vleFluidType;

      SI.AbsolutePressure p "Thermodynamic pressure in the connection point";
      flow SI.MassFlowRate m_flow
        "Mass flow rate from the connection point into the component";
      stream SI.SpecificEnthalpy h_outflow
        "Specific thermodynamic enthalpy close to the connection point if m_flow < 0";
      stream SI.SpecificEnthalpy h_limit;
      stream SI.MassFraction xi_outflow[vleFluidType.nc-1]
        "Independent mixture mass fractions m_i/m close to the connection point if m_flow < 0";

      annotation (Icon(graphics={Ellipse(
              extent={{-80,80},{80,-80}},
              lineColor={153,204,0},
              lineThickness=0.5,
              fillColor={153,204,0},
              fillPattern=FillPattern.Solid)}),
                    Diagram(graphics={Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={153,204,0},
              fillColor={153,204,0},
              fillPattern=FillPattern.Solid), Text(
              extent={{-100,100},{100,60}},
              lineColor={153,204,0},
              fillColor={153,204,0},
              fillPattern=FillPattern.Solid,
              textString =     "%name")}));
    end VLEFluidPort;

    connector HeatPort "Thermal port for 1-dim. heat transfer"
      SI.Temperature T "Port temperature";
      flow SI.HeatFlowRate Q_flow "Heat flow rate";

      annotation (Icon(graphics={Rectangle(
              extent={{-60,60},{60,-60}},
              lineColor={204,0,0},
              lineThickness=0.5,
              fillColor={204,0,0},
              fillPattern=FillPattern.Solid)}),
                                 Diagram(graphics={Rectangle(
              extent={{-40,40},{40,-40}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={204,0,0},
              fillPattern=FillPattern.Solid), Text(
              extent={{-100,100},{100,60}},
              lineColor={204,0,0},
              textString=
                   "%name")}));
    end HeatPort;

    connector RotatoryFlange "1D rotational flange"
      SI.Angle phi( start=0) "Absolute rotation angle of flange";
      flow SI.Torque tau "Cut torque in the flange";

      annotation(defaultComponentName = "rotatoryFlange",Icon(graphics={Ellipse(
              extent={{-80,80},{80,-80}},
              lineColor={135,135,135},
              lineThickness=0.5,
              fillColor={135,135,135},
              fillPattern=FillPattern.Solid)}),
        Diagram(graphics={Text(
              extent={{-100,100},{100,60}},
              lineColor={135,135,135},
              textString=
                   "%name"), Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={135,135,135},
              fillColor={135,135,135},
              fillPattern=FillPattern.Solid)}),
        Terminal(Rectangle(extent=[-100, -100; 100, 100], style(color=0,
                fillColor=10))));

    end RotatoryFlange;

    package Internals
    extends TIL.Internals.ClassTypes.InternalPackage;

      connector dpdtPort
        flow Real dpdt;
        flow Real counter;
      end dpdtPort;

      connector vleFluidMassPort "System information manager port"
        flow SI.Mass vleFluidMass "Accumulated fluid mass";
        flow SI.Volume vleFluidVolume "Accumulated fluid volume";
        annotation(Icon(graphics={Ellipse(
                extent={{-80,80},{80,-80}},
                lineColor={127,127,0},
                fillColor={127,127,0},
                fillPattern=FillPattern.Solid)}));
      end vleFluidMassPort;

      connector liquidMassPort "System information manager port"
        flow SI.Mass liquidMass "Accumulated liquid mass";
        flow SI.Volume liquidVolume "Accumulated liquid volume";
        annotation(Icon(graphics={Ellipse(
                extent={{-80,80},{80,-80}},
                lineColor={127,127,0},
                fillColor={127,127,0},
                fillPattern=FillPattern.Solid)}));
      end liquidMassPort;
    end Internals;
   annotation(classOrder={"VLEFluidPort","GasPort","LiquidPort","SLEMediumPort",
      "VLEFluidPort","GasPort","LiquidPort",
      "HeatPort","HeatPortCold","HeatPort","RotatoryPort","*","Internals"},
      __Dymola_Protection(nestedAllowDuplicate = true,nestedShowDiagram=true,
               nestedShowText=true));
  end Connectors;

  package Utilities "Numeric functions and unit definitions"
   extends TIL.Internals.ClassTypes.ModelPackage;

    package Numerics
    extends TIL.Internals.ClassTypes.ModelPackage;

      function smoothLimiter
       input Real x "actual value that should be limited";
       input Real x_max(min=0) "maximum value";
       parameter Real relativeTransitionWidth(min=0) = 0.9
          "is greater than 0 and smaller than 1. The switching zone is from +-x=x_max*relativeTransitionWidth to +-x=x_max";
       output Real y;
      protected
        Real x_offset;
        Real a;
        Real b;
      algorithm
        x_offset := x_max*relativeTransitionWidth;
        a := x_max*(1-relativeTransitionWidth)/Modelica.Constants.pi*2;
        b := 1/a;
        if abs(x) < x_offset then
          y := x;
        elseif x > 0 then
          y := x_offset + a*atan((x-x_offset)*b);
        else
          y := -x_offset + a*atan((x+x_offset)*b);
        end if;

        annotation (Documentation(info="<html>
<p>function definition:</p>
<p><img src=\"modelica://TIL/Images/equations/equation-qcf2Y0nm.png\" alt=\"f(v)=a*atan(b*(v-v_offset))\"/></p>
<p><br><br>derivative:</p>
<p><img src=\"modelica://TIL/Images/equations/equation-3wPbq3Y7.png\" alt=\"df(v)=a*b*(1/((b*(v-v_offset))^2+1))\"/></p>
<p><br><br>derivative at switching point x_offset:</p>
<p><img src=\"modelica://TIL/Images/equations/equation-7HuHQkrv.png\" alt=\"df(v_offset)=a*b*(1/((b*(0))^2+1))
\"/></p>
<p><br><img src=\"modelica://TIL/Images/equations/equation-GzskhgFd.png\" alt=\"df(v_offset)=a*b\"/></p>
<p><br><br>the value must approach the maximum:</p>
<p><img src=\"modelica://TIL/Images/equations/equation-WOxNqYxZ.png\" alt=\"f(infinity)=v_max-v_offset\"/></p>
<p><img src=\"modelica://TIL/Images/equations/equation-dNo00NvU.png\" alt=\"atan(infinity)=pi/2\"/></p>
<p><img src=\"modelica://TIL/Images/equations/equation-qL9YQABI.png\" alt=\"a*pi/2=v_max-v_offset\"/></p>
<p><img src=\"modelica://TIL/Images/equations/equation-oK4hzi6F.png\" alt=\"a=(v_max-v_offset)/pi*2\"/></p>
<p>since the derivative at the switching point must be zero:</p>
<p><br><img src=\"modelica://TIL/Images/equations/equation-ZpEpu3Gb.png\" alt=\"df(v_offset)=1\"/></p>
<p><img src=\"modelica://TIL/Images/equations/equation-sKamJP1N.png\" alt=\"a*b=1\"/></p>
<p><img src=\"modelica://TIL/Images/equations/equation-HXmgJkqB.png\" alt=\"b=1/a\"/></p>
<pre> 
Plot Skript:

v=linspace(-1,1,1000);
v_limited={TIL.Utilities.Numerics.smoothLimiter(v[i],0.5) for i in 1:size(v,1)};
plotArray(v,v_limited);</pre>
</html>"),Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end smoothLimiter;

      function smoothTransition
        input Real x;
        input Real transitionPoint=1;
        input Real transitionLength=1;
        input Integer funcNum=0 "Function selector";

        output Real weigthingFactor;
      protected
        Real phi "Phase";
      algorithm
        if (x < transitionPoint-0.5*transitionLength) then
          weigthingFactor := 1;
        elseif (x < transitionPoint+0.5*transitionLength) then
          phi := (x - transitionPoint)*Modelica.Constants.pi/transitionLength;
          if (funcNum == 0) then
            weigthingFactor := -1.0/2.0*sin(phi)+1.0/2.0;
          elseif (funcNum == 1) then
            weigthingFactor := -1.0/2.0*(2*cos(phi)*sin(phi) + 2*phi - Modelica.Constants.pi)/
              Modelica.Constants.pi;
          elseif (funcNum == 2) then
            weigthingFactor := 1.0/6.0*(-4.0*sin(phi)*cos(phi)^3 - 6.0*phi - 3.0*sin(2.0*phi) +
              3.0*Modelica.Constants.pi)/Modelica.Constants.pi;
          elseif (funcNum == 3) then
            weigthingFactor := 1.0/30.0*(-16*cos(phi)^5*sin(phi) - 20*cos(phi)^3*sin(phi) - 30*cos(
              phi)*sin(phi) - 30*phi + 15*Modelica.Constants.pi)/Modelica.Constants.pi;
          else
            weigthingFactor := 0;
          end if;
        else
          weigthingFactor := 0;
        end if;

        annotation (derivative(order=1,noDerivative=transitionPoint,
                                       noDerivative=transitionLength)=smoothTransition_der,
            Documentation(info="<html>
<p><img src=\"modelica://TIL/Images/smoothTransition.png\"/></p>
<p><br>This function smoothly changes its output from 1 to 0 around the TransitionPoint with the TransitionLength.</p>
<p><br>The parameter funcNum defines the smoothness of the transition.</p>
<p>With funcNum=0 the transition is continuously. With funcNum=1 the transition is once continuously differentiable. With funcNum=2 the transition is twice continuously differentiable.</p>
</html>"));
      end smoothTransition;

      function smoothTransition_der
        input Real x;
        input Real transitionPoint=1;
        input Real transitionLength=1;
        input Integer funcNum=0 "Function selector";
        input Real x_der;

        output Real weigthingFactor;
      protected
        Real phi "Phase";
        Real dphidt "Phase";
      algorithm
        if (x < transitionPoint-0.5*transitionLength) then
          weigthingFactor := 0;
        elseif (x < transitionPoint+0.5*transitionLength) then
          phi := (x - transitionPoint)*Modelica.Constants.pi/transitionLength;
          dphidt := x_der*Modelica.Constants.pi/transitionLength;
          if (funcNum == 0) then
            weigthingFactor := (-0.5)*(dphidt*cos(phi));
          elseif (funcNum == 1) then
            weigthingFactor :=  -(cos(phi)*(dphidt*cos(phi))-dphidt*sin(phi)*sin(phi)+
              dphidt)/Modelica.Constants.pi;
          elseif (funcNum == 2) then
            weigthingFactor := 1.0/6.0*((-4.0)*(dphidt*cos(phi)*cos(phi)^3-
              3.0*(sin(phi)*(cos(phi)^2*(dphidt*sin(phi)))))-6.0*dphidt-6.0*(dphidt*cos(
              2.0*phi)))/Modelica.Constants.pi;
          elseif (funcNum == 3) then
            weigthingFactor := 1.0/30.0*((-16.0)*(cos(phi)^5*(dphidt*cos(phi))
              -5.0*(cos(phi)^4*(dphidt*sin(phi))*sin(phi)))-20.0*(cos(phi)^3*(dphidt*cos(
              phi))-3.0*(cos(phi)^2*(dphidt*sin(phi))*sin(phi)))-30.0*(cos(phi)*(dphidt*
              cos(phi))-dphidt*sin(phi)*sin(phi))-30*dphidt)/Modelica.Constants.pi;
          else
            weigthingFactor := 0;
          end if;
        else
          weigthingFactor := 0;
        end if;

        annotation (derivative(order=2,noDerivative=transitionPoint,
                                       noDerivative=transitionLength)=smoothTransition_der2);
      end smoothTransition_der;

      function smoothTransition_der2
        input Real x;
        input Real transitionPoint=1;
        input Real transitionLength=1;
        input Integer funcNum=0 "Function selector";
        input Real x_der;
        input Real x_der2;

        output Real weigthingFactor;
      protected
        Real phi "Phase";
        Real dphidt "Phase";
        Real d2phidt2 "Phase";
      algorithm
        if (x < transitionPoint-0.5*transitionLength) then
          weigthingFactor := 0;
        elseif (x < transitionPoint+0.5*transitionLength) then
          phi := (x - transitionPoint)*Modelica.Constants.pi/transitionLength;
          dphidt := Modelica.Constants.pi/transitionLength*x_der;
          d2phidt2 := x_der2*Modelica.Constants.pi/transitionLength;
          if (funcNum == 0) then
            weigthingFactor := (-0.5)*(d2phidt2*cos(phi)-dphidt*(dphidt*sin(phi)));
          elseif (funcNum == 1) then
            weigthingFactor := -(cos(phi)*(d2phidt2*cos(phi)-dphidt*(dphidt*sin(phi)))
              -dphidt*sin(phi)*(dphidt*cos(phi))-((d2phidt2*sin(phi)+dphidt*(dphidt*cos(
               phi)))*sin(phi)+dphidt*sin(phi)*(dphidt*cos(phi)))+d2phidt2)/
              Modelica.Constants.pi;
          elseif (funcNum == 2) then
            weigthingFactor := 1.0/6.0*((-4.0)*((d2phidt2*cos(phi)-
              dphidt*(dphidt*sin(phi)))*cos(phi)^3-3.0*(dphidt*cos(phi)*(cos(phi)^2*(
              dphidt*sin(phi))))-3.0*(dphidt*cos(phi)*(cos(phi)^2*(dphidt*sin(phi)))+sin(
              phi)*(cos(phi)^2*(d2phidt2*sin(phi)+dphidt*(dphidt*cos(phi)))-2.0*(cos(
              phi)*(dphidt*sin(phi))*(dphidt*sin(phi))))))-6.0*d2phidt2-6.0*(
              d2phidt2*cos(2.0*phi)-2.0*(dphidt*(dphidt*sin(2.0*phi)))))/
              Modelica.Constants.pi;
          elseif (funcNum == 3) then
            weigthingFactor := 1.0/30.0*((-16.0)*(cos(phi)^5*(d2phidt2*
              cos(phi)-dphidt*(dphidt*sin(phi)))-5.0*(cos(phi)^4*(dphidt*sin(phi))*(dphidt
              *cos(phi)))-5.0*((cos(phi)^4*(d2phidt2*sin(phi)+dphidt*(dphidt*cos(phi)))
              -4.0*(cos(phi)^3*(dphidt*sin(phi))*(dphidt*sin(phi))))*sin(phi)+cos(phi)^4*(
              dphidt*sin(phi))*(dphidt*cos(phi))))-20.0*(cos(phi)^3*(d2phidt2*cos(phi)-
              dphidt*(dphidt*sin(phi)))-3.0*(cos(phi)^2*(dphidt*sin(phi))*(dphidt*cos(phi)))
              -3.0*((cos(phi)^2*(d2phidt2*sin(phi)+dphidt*(dphidt*cos(phi)))-2.0*(cos(
              phi)*(dphidt*sin(phi))*(dphidt*sin(phi))))*sin(phi)+cos(phi)^2*(dphidt*sin(
              phi))*(dphidt*cos(phi))))-30.0*(cos(phi)*(d2phidt2*cos(phi)-dphidt*(
              dphidt*sin(phi)))-dphidt*sin(phi)*(dphidt*cos(phi))-((d2phidt2*sin(phi)+
              dphidt*(dphidt*cos(phi)))*sin(phi)+dphidt*sin(phi)*(dphidt*cos(phi))))-30*
              d2phidt2)/Modelica.Constants.pi;
          else
            weigthingFactor := 0;
          end if;
        else
          weigthingFactor := 0;
        end if;
      end smoothTransition_der2;

      function squareRootFunction
        input Real x;
        input Real delta_x = 1e-4;
        output Real y;

      protected
      Real a;
      Real b;

      algorithm
        a := -1/(4*delta_x^(5/2));
        b := 5/(4*sqrt(delta_x));

      if (x > delta_x) then
        y := sqrt(x);
      elseif (x < -delta_x) then
        y := -sqrt(-x);
      else
        y := (a*x*x + b)*x;
      end if;

      annotation(smoothOrder=1, inverse(x = TIL.Utilities.Numerics.inverseSquareRootFunction(y,delta_x)));
      end squareRootFunction;

      function inverseSquareRootFunction
        input Real x;
        input Real delta_y = 1e-4;
        output Real y;

      protected
      Real a;
      Real b;

      Real delta_x;

      algorithm
        delta_x := sqrt(delta_y);

        a := 0.5/delta_x;
        b := 0.5*delta_x;

      if (x > delta_x) then
        y := x*x;
      elseif (x < -delta_x) then
        y := -x*x;
      else
        y := (a*x*x + b)*x;
      end if;

      annotation(smoothOrder=1);
      end inverseSquareRootFunction;
      annotation(classOrder={"*","Testers"});
    end Numerics;

    package Units "Additionaly required unit definitions for TIL"
    extends TIL.Internals.ClassTypes.ModelPackage;

      type EntropyFlowRate = Real(final unit="W/K");

      type MassFlowDensity = Real(final unit="kg/(s.m2)");

      type RelativeVolume = Real (
          final quantity="RelativeVolume",
          final unit="1",
          min=0);
    end Units;
  end Utilities;

  package Internals
  "Internal types, models and functions used in the component models"

    type PressureStateID
      extends Integer(min=0);
    end PressureStateID;

    type BoundaryType
      extends String;
      annotation(choices(
        choice="p" "p - Pressure is fixed",
        choice="m_flow" "m_flow - Mass flow rate is fixed",
        choice="V_flow" "V_flow - Volume flow rate is fixed"));
    end BoundaryType;

    type CellFlowType
      extends String;
       annotation(choices(
        choice="allow reverse flow"
            "flow direction A-B and B-A with m_flow=0 is allowed",
        choice="flow A-B" " only flow direction A-B is allowed",
        choice="flow B-A" " only flow direction B-A is allowed"));
    end CellFlowType;

    package ClassTypes "Icon definitions"

      partial class ComponentPackage

        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,-100},{80,50}},
                lineColor={0,0,0},
                fillColor={153,204,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
                lineColor={0,0,0},
                fillColor={153,204,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
                lineColor={0,0,0},
                fillColor={153,204,0},
                fillPattern=FillPattern.Solid)}),
          classOrder={"*", "Internals","Testers"});

      end ComponentPackage;

      partial class ExamplePackage

        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,-100},{80,50}},
                lineColor={0,0,0},
                fillColor={204,0,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
                lineColor={0,0,0},
                fillColor={204,0,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
                lineColor={0,0,0},
                fillColor={204,0,0},
                fillPattern=FillPattern.Solid)}));

      end ExamplePackage;

      partial class InternalPackage

        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,-100},{80,50}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end InternalPackage;

      partial class ModelPackage

       annotation (Icon(graphics={
              Rectangle(
                extent={{-100,-100},{80,50}},
                lineColor={0,0,0},
                fillColor={255,204,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
                lineColor={0,0,0},
                fillColor={255,204,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
                lineColor={0,0,0},
                fillColor={255,204,0},
                fillPattern=FillPattern.Solid)}));
      end ModelPackage;

      partial record Record "Partial Record"

        annotation (Icon(graphics={        Text(
                extent={{-100,110},{100,70}},
                lineColor={0,0,0},
                textString=
                     "%name"), Bitmap(
                extent={{-100,-100},{100,60}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAMgAAADICAIAAAAiOjnJAAAABnRSTlMA/wAAAACkwsAdAAAACXBIWXMAAAsTAAALEwEAmpwYAAACp0lEQVR42u3cMU4CQRSA4V2lVDsDYTmMpbUJcACtSTgJ6i2AS1ChHoAbQELCFSgsKCxNXJ7uW7+/32Qz8+VNsztlv1sVanyj8Wj2Okv0wpfXVze2rfltNpvt49P9x3uWFy5PE2u3367Xb/avmQ0fhunm1hcs+9fYqt4g3Zl4YdtytZgvppMpWDrfgTgeJrIFVpqeX2aJbIHFFljKYwsstsBSHltgsQWW8tgCiy2wlMcWWGyBpTy2wGILLOWxBRZbIXXqPLw6HG1kM20VRbGcL0+2isn0978NNLHMLbCUxxZYbIGlPLbAYgss5bEFFltgKY8tsNgCS3lsgcVWiC2w2AqxBRZbIbbAYivEFlhshdgCSyG2wFKILbAUYgsshdgCSyG2wFKILbAUYgsshdgCSyG2wFKILbAUYgsshdgCSyG2wFKILbAUYgsshdgCSyG2wFKILbAUYgsshdgCSyG2wFKIrY51yVLVG6R4z9M9zSaWQmyV/W5VFMVuv7UcOuNkNbEUElgCS2AJLAksgSWwJLAElsCSwBJYAksCS2AJLAksgaX/Vq2/dFaHoxVscXe3P+dhYslRKLAElgSWwBJYElgCS2BJYAksgSWBJbAElgSWwBJYElgCS2BJYAksgSWBJbAElgSWwBJYElgCS2BJYAksgSWBJbDU4mrd817nHnCZWBJYAktgSWAJLIElgSWwBJYElsASWBJYAktgSWAJLIElfVOtj9ZXh6MVbHF1/mkwseQoFFgCSwJLYAksCSyBJbAksASWwJLAElgCSwJLYAksCSyBJbAksASWwJLAElgCSwJLYAksCSyBJbAksASWwJLAElhqcbXuea9zD7hMLAksgSWwJLAElsCSwBJYAksCS2AJLAksgSWwJLD0R5X9bmUVZGIpR5/i9W1YEPGgigAAAABJRU5ErkJggg==",
                fileName="modelica://TIL/Images/Record.png")}));

      end Record;
      annotation (Icon(graphics={
            Rectangle(
              extent={{-100,-100},{80,50}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end ClassTypes;

    model GetInputsHydraulic "Get enabled inputs and parameters of disabled inputs"
      extends Modelica.Blocks.Interfaces.BlockIcon;

      Modelica.Blocks.Interfaces.RealInput m_flow_in "Prescribed mass flow rate"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}},
              rotation=0)));
      Modelica.Blocks.Interfaces.RealInput V_flow_in "Prescribed volume flow rate"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}},
              rotation=0)));
      Modelica.Blocks.Interfaces.RealInput dp_in "Prescribed pressure increase"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}}, rotation=
               0)));

      annotation (Diagram(graphics));
    end GetInputsHydraulic;

    model GetInputsRotary "Get enabled inputs and parameters of disabled inputs"
      extends Modelica.Blocks.Interfaces.BlockIcon;

      Connectors.RotatoryFlange rotatoryFlange
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=
               0)));
      annotation (Diagram(graphics));
    end GetInputsRotary;

    model GetInputsThermal "Get enabled inputs and parameters of disabled inputs"
      extends Modelica.Blocks.Interfaces.BlockIcon;

      Connectors.HeatPort heatPort annotation (Placement(transformation(extent={{
                -110,-10},{-90,10}}, rotation=0)));
      annotation (Diagram(graphics));
    end GetInputsThermal;

    type InitializationMethods
      extends String;
      annotation(Evaluate=true,choices(
        choice="constantEnthalpy",
        choice="steadyState",
        choice="linearEnthalpyDistribution",
        choice="constantTemperature",
        choice="userSpecifiedValues"));
    end InitializationMethods;

    type JunctionFlowType
      extends String;
       annotation(choices(
        choice="allow reverse flow" "allow reverse flow",
        choice="flow B-AC" "only flow direction B-AC is allowed",
        choice="flow AC-B" "only flow direction AC-B is allowed"));
    end JunctionFlowType;

    model SimPort
      Real vleFluidMass=vleFluidPort.vleFluidMass;
      Real vleFluidVolume=vleFluidPort.vleFluidVolume;
      Real liquidMass=liquidPort.liquidMass;
      Real liquidVolume=liquidPort.liquidVolume;
      Real dpdt=dpdtPort.dpdt;
      Real dpdtCounter=dpdtPort.counter;
      TIL.Connectors.Internals.vleFluidMassPort vleFluidPort;
      TIL.Connectors.Internals.liquidMassPort liquidPort;
      TIL.Connectors.Internals.dpdtPort dpdtPort;
    end SimPort;

    type PresetVariableType
      extends String;
      annotation(choices(
        choice="dp" "dp",
        choice="m_flow" "m_flow",
        choice="V_flow" "V_flow"));
    end PresetVariableType;

    type AlphaAStateChoice = enumeration(
        useAlphaAState "force on",
        doNotUseAlphaAState "force off",
        definedByHeatTransferCorrelation "defined by heat transfer");

    function getPortValuesStirredVolumeCells
      input Real value_portA "flow dependent value at portA";
      input Real value_Array[:] "inStream values in cells";
      input Real value_portB "flow dependent value at portB";
      input Real m_flow_Array[size(value_Array,1)-1]
        "mass flow rates between cells (portA)";
      output Real portValueArray[size(value_Array,1)+1];
    algorithm
      portValueArray[1]:=value_portA;
      portValueArray[size(value_Array,1)+1]:=value_portB;
      for i in 2:(size(value_Array,1)) loop
        portValueArray[i]:=if (m_flow_Array[i-1] >=0) then value_Array[i-1] else value_Array[i];
      end for;
      annotation(Inline=true);
    end getPortValuesStirredVolumeCells;

    type gasPressureOrientation
      extends String;
       annotation(choices(
        choice="A" "pressure result from portA",
        choice="B" "pressure result from portB",
        choice="symmetric" "mean pressure between portA and portB is used"));
    end gasPressureOrientation;
  annotation(classOrder={"GasID","LiquidID","VLEFluidID","SLEMediumID","PressureStateID","ComponentType","PackageTypes","*","Internals"}, Icon(
        graphics={
          Rectangle(
            extent={{-100,-100},{80,50}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end Internals;
  annotation (
  Icon(coordinateSystem(
      preserveAspectRatio=false,
      extent={{-100,-100},{100,100}},
      grid={1,1},
      initialScale=0.1), graphics={Bitmap(extent={{-100,-100},{100,100}},
        imageSource=
            "iVBORw0KGgoAAAANSUhEUgAAAgAAAAIACAYAAAD0eNT6AAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAZdEVYdFNvZnR3YXJlAEFkb2JlIEltYWdlUmVhZHlxyWU8AAADaGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4gPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS4wLWMwNjEgNjQuMTQwOTQ5LCAyMDEwLzEyLzA3LTEwOjU3OjAxICAgICAgICAiPiA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPiA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIiB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIgeG1sbnM6c3RSZWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZVJlZiMiIHhtbG5zOnhtcD0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wLyIgeG1wTU06T3JpZ2luYWxEb2N1bWVudElEPSJ4bXAuZGlkOkY3N0YxMTc0MDcyMDY4MTE5N0E1ODZBOEE1QTEwMDA3IiB4bXBNTTpEb2N1bWVudElEPSJ4bXAuZGlkOjg4RjQwNEY0NEU0QzExRTE5RjAxRjk3Q0VCNTBEMDI5IiB4bXBNTTpJbnN0YW5jZUlEPSJ4bXAuaWlkOjg4RjQwNEYzNEU0QzExRTE5RjAxRjk3Q0VCNTBEMDI5IiB4bXA6Q3JlYXRvclRvb2w9IkFkb2JlIFBob3Rvc2hvcCBDUzUuMSBNYWNpbnRvc2giPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDpCQkI2RDg3QjI2MjA2ODExOEE2REUwQzAzNjQwQzhCMCIgc3RSZWY6ZG9jdW1lbnRJRD0ieG1wLmRpZDpGNzdGMTE3NDA3MjA2ODExOTdB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"
             +
            "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",
        fileName="modelica://TIL/Images/TIL.png")}),
             uses(TILMedia(version="3.4.1"), Modelica(version="3.2.2")),
preferedView="info",
version="3.4.1",
Documentation(info="<html><br>
<img src=\"modelica://TIL/Images/InfoTIL.png\"><br>
<hr>
</html>", revisions=""),   Icon(
      Text(
        extent=[-120,144; 120,95],
        string="%name",
        style(color=1))),
conversion(
 from(version="2.0.0", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.0.1", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.0.2", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.0.3", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.0.4", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.0.5", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.1.0", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.1.1", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="2.1.2", script="Scripts/ConvertTIL_to_3.0.0.mos"),
 from(version="3.0.0", script="Scripts/ConvertTIL_from_3.0.x.mos"),
 from(version="3.0.1", script="Scripts/ConvertTIL_from_3.0.x.mos"),
 from(version="3.0.2", script="Scripts/ConvertTIL_from_3.0.x.mos"),
 from(version="3.1.0", script="Scripts/ConvertTIL_from_3.1.0.mos"),
 from(version="3.2.0", script="Scripts/ConvertTIL_from_3.2.x.mos"),
 from(version="3.2.1", script="Scripts/ConvertTIL_from_3.2.x.mos"),
 from(version="3.2.2", script="Scripts/ConvertTIL_from_3.2.2.mos"),
 from(version="3.2.3", script="Scripts/ConvertTIL_from_3.2.3.mos"),
 from(version="3.4.0", script="Scripts/ConvertTIL_from_3.4.0.mos")));
end TIL;

package TILMedia
  "TILMedia-Library with thermophysical properties of Fluids and Solids"
  import SI = Modelica.SIunits;

  model Gas_ph "Gas vapor model with p, h and xi as independent variables"
    replaceable parameter TILMedia.GasTypes.BaseGas gasType
      constrainedby TILMedia.GasTypes.BaseGas
      "type record of the gas or gas mixture"
      annotation(choicesAllMatching=true);

    parameter Boolean stateSelectPreferForInputs=false
      "=true, StateSelect.prefer is set for input variables"
      annotation(Evaluate=true,Dialog(tab="Advanced",group "StateSelect"));
    parameter Boolean computeTransportProperties = false
      "=true, if transport properties are calculated"
      annotation(Dialog(tab="Advanced"));

    //Base Properties
    SI.Density d "Density";
    input SI.SpecificEnthalpy h(stateSelect=if (stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Specific enthalpy" annotation(Dialog);
    input SI.AbsolutePressure p(stateSelect=if (stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Pressure" annotation(Dialog);
    SI.SpecificEntropy s "Specific entropy";
    SI.Temperature T "Temperature";
    input SI.MassFraction xi[gasType.nc-1](stateSelect=if (stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default) = gasType.xi_default
      "Mass fraction" annotation(Dialog);
    SI.MassFraction xi_dryGas[if (gasType.nc>1 and gasType.condensingIndex>0) then gasType.nc-2 else 0]
      "Mass fraction";
    SI.MoleFraction x[gasType.nc-1] "Mole fraction";
    SI.MolarMass M "Average molar mass";

    //Additional Properties
    SI.SpecificHeatCapacity cp "Specific isobaric heat capacity cp";
    SI.SpecificHeatCapacity cv "Specific isochoric heat capacity cv";
    SI.LinearExpansionCoefficient beta "Isobaric thermal expansion coefficient";
    SI.Compressibility kappa "Isothermal compressibility";
    SI.Velocity w "Speed of sound";
    SI.DerDensityByEnthalpy drhodh_pxi
      "Derivative of density wrt specific enthalpy at constant pressure and mass fraction";
    SI.DerDensityByPressure drhodp_hxi
      "Derivative of density wrt pressure at specific enthalpy and mass fraction";
    TILMedia.Internals.Units.DensityDerMassFraction drhodxi_ph[gasType.nc-1]
      "Derivative of density wrt mass fraction of water at constant pressure and specific enthalpy";
    SI.PartialPressure p_i[gasType.nc] "Partial pressure";
    SI.MassFraction xi_gas "Mass fraction of gasoues condensing component";
    TILMedia.Internals.Units.RelativeHumidity phi(min=if (gasType.condensingIndex>0) then 0 else -1)
      "Relative humidity";

    //Pure Component Properties
    SI.PartialPressure p_s "Saturation partial pressure of condensing component";
    SI.MassFraction xi_s(min=if (gasType.condensingIndex>0) then 0 else -1)
      "Saturation mass fraction of condensing component";
    SI.SpecificEnthalpy delta_hv
      "Specific enthalpy of vaporation of condensing component";
    SI.SpecificEnthalpy delta_hd
      "Specific enthalpy of desublimation of condensing component";
    SI.SpecificEnthalpy h_i[gasType.nc]
      "Specific enthalpy of theoretical pure component";
    SI.MolarMass M_i[gasType.nc] "Molar mass of component i";

    //Dry Component Specific Properties
    Real humRatio "Content of condensing component aka humidity ratio";
    Real humRatio_s
      "Saturation content of condensing component aka saturation humidity ratio";
    SI.SpecificEnthalpy h1px
      "Enthalpy H divided by the mass of components that cannot condense";

    TILMedia.Internals.TransportPropertyRecord transp "Transport property record" annotation (extent=[-80,40; -60,60]);

    TILMedia.GasObjectFunctions.GasPointer gasPointer=TILMedia.GasObjectFunctions.GasPointer(gasType.concatGasName, computeFlags, gasType.mixingRatio_propertyCalculation[1:end-1]/sum(gasType.mixingRatio_propertyCalculation), gasType.nc_propertyCalculation, gasType.nc, gasType.condensingIndex, redirectorOutput)
      "Pointer to external medium memory";
  protected
    constant Real invalidValue=-1;
    final parameter Integer computeFlags = TILMedia.Internals.calcComputeFlags(computeTransportProperties,false,true,false);
    parameter Integer redirectorOutput=TILMedia.Internals.redirectModelicaFormatMessage();
  equation
    //calculate molar mass
    M = 1/sum(cat(1,xi,{1-sum(xi)})./M_i);
    //calculate molar fraction
    xi = x.*M_i[1:end-1]*(sum(cat(1,xi,{1-sum(xi)})./M_i)); //xi = x.*M_i/M
    //calculate relative humidity, water content, h1px
    if (gasType.condensingIndex>0 and gasType.nc>1) then
      if (gasType.condensingIndex==gasType.nc) then
        cat(1,xi_dryGas,{1-sum(xi_dryGas)})=xi*(1+humRatio);
      else
        humRatio = xi[gasType.condensingIndex]*(humRatio+1);
        for i in 1:gasType.nc-1 loop
          if (i <> gasType.condensingIndex) then
            xi_dryGas[if (i<gasType.condensingIndex) then i else i-1] = xi[i]*(humRatio+1);
          end if;
        end for;
      end if;
      h1px = h*(1+humRatio);
      phi=TILMedia.Internals.GasObjectFunctions.phi_pThumRatioxidg(p,T,humRatio,xi_dryGas,gasPointer);
      humRatio_s = TILMedia.Internals.GasObjectFunctions.humRatio_s_pTxidg(p, T, xi_dryGas, gasPointer);
      xi_s = TILMedia.Internals.GasObjectFunctions.xi_s_pTxidg(p, T, xi_dryGas, gasPointer);
    else
      phi = -1;
      humRatio = -1;
      h1px = -1;
      humRatio_s = -1;
      xi_s = -1;
    end if;

    if (gasType.condensingIndex<=0) then
      // some properties are only pressure dependent if there is vapour in the mixture
      T = TILMedia.Internals.GasObjectFunctions.temperature_phxi(-1, h, xi, gasPointer);
      (cp, cv, beta, w) = TILMedia.Internals.GasObjectFunctions.simpleCondensingProperties_phxi(-1, h, xi, gasPointer);
    else
      T = TILMedia.Internals.GasObjectFunctions.temperature_phxi(p, h, xi, gasPointer);
      (cp, cv, beta, w) = TILMedia.Internals.GasObjectFunctions.simpleCondensingProperties_phxi(p, h, xi, gasPointer);
    end if;
    s = TILMedia.Internals.GasObjectFunctions.specificEntropy_phxi(p, h, xi, gasPointer);
    M_i = TILMedia.GasObjectFunctions.molarMass_n({i-1 for i in 1:gasType.nc},gasPointer);
    (d,kappa,drhodp_hxi,drhodh_pxi,drhodxi_ph,p_i,xi_gas) = TILMedia.Internals.GasObjectFunctions.additionalProperties_pTxi(p,T,xi,gasPointer);
    (p_s,delta_hv,delta_hd,h_i) = TILMedia.Internals.GasObjectFunctions.pureComponentProperties_Tnc(T,gasType.nc,gasPointer);
    if computeTransportProperties then
      transp = TILMedia.Internals.GasObjectFunctions.transportProperties_pTxi(p, T, xi, gasPointer);
    else
      transp = TILMedia.Internals.TransportPropertyRecord(
        invalidValue,
        invalidValue,
        invalidValue,
        invalidValue);
    end if;

    annotation (defaultComponentName="gas", Icon(graphics={Bitmap(extent={{-100,
                -100},{100,100}},
            imageSource=
                "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",
            fileName="modelica://TILMedia/Images/Gas_ph.png"),
                     Text(extent={{-120,-60},{120,-100}},lineColor={255,153,0},textString = "%name")}),
                     Documentation(info="<html>
                   <p>
                   The gas model Gas_ph calculates the thermopyhsical property data with given inputs: pressure (p), enthalpy (h), massfraction (xi) and the parameter gasType.<br>
                   The interface and the way of using, is demonstrated in the Testers -> <a href=\"Modelica:TILMedia.Testers.TestGas\">TestGas</a>.
                   </p>
                   <hr>
                   </html>"));
  end Gas_ph;

  model Gas_pT "Gas vapor model with p, T and xi as independent variables"
    replaceable parameter TILMedia.GasTypes.BaseGas gasType constrainedby
      TILMedia.GasTypes.BaseGas "type record of the gas or gas mixture"
      annotation(choicesAllMatching=true);

    parameter Boolean stateSelectPreferForInputs=false
      "=true, StateSelect.prefer is set for input variables"
      annotation(Evaluate=true,Dialog(tab="Advanced",group "StateSelect"));
    parameter Boolean computeTransportProperties = false
      "=true, if transport properties are calculated"
      annotation(Dialog(tab="Advanced"));

    //Base Properties
    SI.Density d "Density";
    SI.SpecificEnthalpy h "Specific enthalpy";
    input SI.AbsolutePressure p(stateSelect=if (stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Pressure" annotation(Dialog);
    SI.SpecificEntropy s "Specific entropy";
    input SI.Temperature T(stateSelect=if (stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Temperature" annotation(Dialog);
    input SI.MassFraction xi[gasType.nc-1](stateSelect=if (stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default) = gasType.xi_default
      "Mass fraction" annotation(Dialog);
    SI.MassFraction xi_dryGas[if (gasType.nc>1 and gasType.condensingIndex>0) then gasType.nc-2 else 0]
      "Mass fraction";
    SI.MoleFraction x[gasType.nc-1] "Mole fraction";
    SI.MolarMass M(min=1e-99) "Average molar mass";

    //Additional Properties
    SI.SpecificHeatCapacity cp "Specific isobaric heat capacity cp";
    SI.SpecificHeatCapacity cv "Specific isochoric heat capacity cv";
    SI.LinearExpansionCoefficient beta "Isobaric thermal expansion coefficient";
    SI.Compressibility kappa "Isothermal compressibility";
    SI.Velocity w "Speed of sound";
    SI.DerDensityByEnthalpy drhodh_pxi
      "Derivative of density wrt specific enthalpy at constant pressure and mass fraction";
    SI.DerDensityByPressure drhodp_hxi
      "Derivative of density wrt pressure at specific enthalpy and mass fraction";
    TILMedia.Internals.Units.DensityDerMassFraction drhodxi_ph[gasType.nc-1]
      "Derivative of density wrt mass fraction of water at constant pressure and specific enthalpy";
    SI.PartialPressure p_i[gasType.nc] "Partial pressure";
    SI.MassFraction xi_gas "Mass fraction of gasoues condensing component";
    TILMedia.Internals.Units.RelativeHumidity phi(min=if (gasType.condensingIndex>0) then 0 else -2)
      "Relative humidity";

    //Pure Component Properties
    SI.PartialPressure p_s "Saturation partial pressure of condensing component";
    SI.MassFraction xi_s(min=if (gasType.condensingIndex>0) then 0 else -1)
      "Saturation mass fraction of condensing component";
    SI.SpecificEnthalpy delta_hv
      "Specific enthalpy of vaporation of condensing component";
    SI.SpecificEnthalpy delta_hd
      "Specific enthalpy of desublimation of condensing component";
    SI.SpecificEnthalpy h_i[gasType.nc]
      "Specific enthalpy of theoretical pure component";
    SI.MolarMass M_i[gasType.nc] "Molar mass of component i";

    //Dry Component Specific Properties
    Real humRatio "Content of condensing component aka humidity ratio";
    Real humRatio_s
      "Saturation content of condensing component aka saturation humidity ratio";
    SI.SpecificEnthalpy h1px
      "Enthalpy H divided by the mass of components that cannot condense";

    TILMedia.Internals.TransportPropertyRecord transp "Transport property record" annotation (extent=[-80,40; -60,60]);

    TILMedia.GasObjectFunctions.GasPointer gasPointer=TILMedia.GasObjectFunctions.GasPointer(gasType.concatGasName, computeFlags, gasType.mixingRatio_propertyCalculation[1:end-1]/sum(gasType.mixingRatio_propertyCalculation), gasType.nc_propertyCalculation, gasType.nc, gasType.condensingIndex, redirectorOutput)
      "Pointer to external medium memory";
  protected
    constant Real invalidValue=-1;
    final parameter Integer computeFlags = TILMedia.Internals.calcComputeFlags(computeTransportProperties,false,true,false);
    parameter Integer redirectorOutput=TILMedia.Internals.redirectModelicaFormatMessage();
  equation
    //calculate molar mass
    M = 1/sum(cat(1,xi,{1-sum(xi)})./M_i);
    //calculate molar fraction
    xi = x.*M_i[1:end-1]*(sum(cat(1,xi,{1-sum(xi)})./M_i)); //xi = x.*M_i/M
    //calculate relative humidity, water content, h1px
    if (gasType.condensingIndex>0 and gasType.nc>1) then
      if (gasType.condensingIndex==gasType.nc) then
        cat(1,xi_dryGas,{1-sum(xi_dryGas)})=xi*(1+humRatio);
      else
        humRatio = xi[gasType.condensingIndex]*(humRatio+1);
        for i in 1:gasType.nc-1 loop
          if (i <> gasType.condensingIndex) then
            xi_dryGas[if (i<gasType.condensingIndex) then i else i-1] = xi[i]*(humRatio+1);
          end if;
        end for;
      end if;
      h1px = h*(1+humRatio);
      phi=TILMedia.Internals.GasObjectFunctions.phi_pThumRatioxidg(p,T,humRatio,xi_dryGas,gasPointer);
      humRatio_s = TILMedia.Internals.GasObjectFunctions.humRatio_s_pTxidg(p, T, xi_dryGas, gasPointer);
      xi_s = TILMedia.Internals.GasObjectFunctions.xi_s_pTxidg(p, T, xi_dryGas, gasPointer);
    else
      phi = -1;
      humRatio = -1;
      h1px = -1;
      humRatio_s = -1;
      xi_s = -1;
    end if;

    if (gasType.condensingIndex<=0) then
      // some properties are only pressure dependent if there is vapour in the mixture
      h = TILMedia.Internals.GasObjectFunctions.specificEnthalpy_pTxi(-1, T, xi, gasPointer);
      (cp, cv, beta, w) = TILMedia.Internals.GasObjectFunctions.simpleCondensingProperties_pTxi(-1, T, xi, gasPointer);
    else
      h = TILMedia.Internals.GasObjectFunctions.specificEnthalpy_pTxi(p, T, xi, gasPointer);
      (cp, cv, beta, w) = TILMedia.Internals.GasObjectFunctions.simpleCondensingProperties_pTxi(p, T, xi, gasPointer);
    end if;
    s = TILMedia.Internals.GasObjectFunctions.specificEntropy_pTxi(p, T, xi, gasPointer);
    M_i = TILMedia.GasObjectFunctions.molarMass_n({i-1 for i in 1:gasType.nc},gasPointer);
    (d,kappa,drhodp_hxi,drhodh_pxi,drhodxi_ph,p_i,xi_gas) = TILMedia.Internals.GasObjectFunctions.additionalProperties_pTxi(p,T,xi,gasPointer);
    (p_s,delta_hv,delta_hd,h_i) = TILMedia.Internals.GasObjectFunctions.pureComponentProperties_Tnc(T,gasType.nc,gasPointer);
    if computeTransportProperties then
      transp = TILMedia.Internals.GasObjectFunctions.transportProperties_pTxi(p, T, xi, gasPointer);
    else
      transp = TILMedia.Internals.TransportPropertyRecord(
        invalidValue,
        invalidValue,
        invalidValue,
        invalidValue);
    end if;

    annotation (defaultComponentName="gas", Icon(graphics={Bitmap(extent={{-100,
                -100},{100,100}},
            imageSource=
                "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",
            fileName="modelica://TILMedia/Images/Gas_pT.png"),
                     Text(extent={{-120,-60},{120,-100}},lineColor={255,153,0},textString = "%name")}),
                     Documentation(info="<html>
                   <p>
                   The gas model Gas_pT calculates the thermopyhsical property data with given inputs: pressure (p), temperature (T), massfraction (xi) and the parameter gasType.<br>
                   The interface and the way of using, is demonstrated in the Testers -> <a href=\"Modelica:TILMedia.Testers.TestGas\">TestGas</a>.
                   </p>
                   <hr>
                   </html>"));
  end Gas_pT;

  model Solid "Solid model with p as independent variable"

    replaceable model SolidType = TILMedia.SolidTypes.BaseSolid constrainedby
      TILMedia.SolidTypes.BaseSolid "type record of the solid"
      annotation(choicesAllMatching=true);

    constant SI.Density d = solid.d "Density";
    input SI.Temperature T "Temperature" annotation(Dialog);
    SI.SpecificHeatCapacity cp "Heat capacity";
    SI.ThermalConductivity lambda "Thermal conductivity";
    constant SI.SpecificHeatCapacity cp_nominal=solid.cp_nominal
      "Specific heat capacity at standard reference point";
    constant SI.ThermalConductivity lambda_nominal=solid.lambda_nominal
      "Thermal conductivity at standard reference point";
  protected
    SolidType solid(final T=T);
  equation
     cp = solid.cp;
     lambda = solid.lambda;

    annotation (defaultComponentName="solid", Icon(graphics={Text(
            extent={{-120,-60},{120,-100}},
            lineColor={135,135,135},
            textString=
                 "%name"), Bitmap(
            extent={{-100,-100},{100,100}},
            imageSource=
                "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",
            fileName="modelica://TILMedia/Images/Solid_T.png")}),
            DymolaStoredErrors,
            Documentation(info="<html>
                   <p>
                   The solid model calculates the thermopyhsical property data with given input: temperature (T) and the model SolidType.<br>
                   The interface and the way of using, is demonstrated in the Testers -> <a href=\"Modelica:TILMedia.Testers.TestSolid\">TestSolid</a>.
                   </p>
                   </html>"));
  end Solid;

  model VLEFluid_ph
    "VLE-Fluid model describing super-critical, subcooled, superheated fluid including the vapour liquid equilibrium (p, h and xi as independent variables)"
    replaceable parameter TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType
      constrainedby TILMedia.VLEFluidTypes.BaseVLEFluid
      "type record of the VLE fluid or VLE fluid mixture"
      annotation (choicesAllMatching=true);

    parameter Boolean stateSelectPreferForInputs=false
      "=true, StateSelect.prefer is set for input variables"
      annotation (Evaluate=true, Dialog(tab="Advanced", group "StateSelect"));
    parameter Boolean computeTransportProperties=false
      "=true, if transport properties are calculated";
    parameter Boolean interpolateTransportProperties=true
      "Interpolate transport properties in vapor dome"
      annotation (Dialog(tab="Advanced"));
    parameter Boolean computeSurfaceTension=true
      annotation (Dialog(tab="Advanced"));
    parameter Boolean computeVLEAdditionalProperties=false
      "Compute detailed vapour liquid equilibrium properties"
      annotation (Evaluate=true);
    parameter Boolean computeVLETransportProperties=false
      "Compute detailed vapour liquid equilibrium transport properties"
      annotation (Evaluate=true);
    parameter Boolean deactivateTwoPhaseRegion=false
      "Deactivate calculation of two phase region"
      annotation (Evaluate=true);

    //Base Properties
    Modelica.SIunits.Density d "Density";
    input Modelica.SIunits.SpecificEnthalpy h(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Specific enthalpy" annotation(Dialog);
    input Modelica.SIunits.AbsolutePressure p(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Pressure" annotation(Dialog);
    Modelica.SIunits.SpecificEntropy s "Specific entropy";
    Modelica.SIunits.Temperature T "Temperature";
    input Modelica.SIunits.MassFraction[vleFluidType.nc - 1] xi(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)=vleFluidType.xi_default
      "Mass Fraction of Component i" annotation(Dialog);
    SI.MoleFraction x[vleFluidType.nc - 1] "Mole fraction";
    SI.MolarMass M "Average molar mass";

    //Additional Properties
    SI.MassFraction q "Steam mass fraction (quality)";
    SI.SpecificHeatCapacity cp "Specific isobaric heat capacity cp";
    SI.SpecificHeatCapacity cv "Specific isochoric heat capacity cv";
    SI.LinearExpansionCoefficient beta "Isobaric thermal expansion coefficient";
    SI.Compressibility kappa "Isothermal compressibility";
    SI.Velocity w "Speed of sound";
    SI.DerDensityByEnthalpy drhodh_pxi
      "1st derivative of density wrt specific enthalpy at constant pressure and mass fraction";
    SI.DerDensityByPressure drhodp_hxi
      "1st derivative of density wrt pressure at specific enthalpy and mass fraction";
    TILMedia.Internals.Units.DensityDerMassFraction drhodxi_ph[vleFluidType.nc - 1]
      "1st derivative of density wrt mass fraction of water at constant pressure and specific enthalpy";
    Real gamma "Heat capacity ratio aka isentropic expansion factor";

    SI.MolarMass M_i[vleFluidType.nc] "Molar mass of component i";

    TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer=
        TILMedia.VLEFluidObjectFunctions.VLEFluidPointer(
        vleFluidType.concatVLEFluidName,
        computeFlags,
        vleFluidType.mixingRatio_propertyCalculation[1:end-1]/sum(vleFluidType.mixingRatio_propertyCalculation),
        vleFluidType.nc_propertyCalculation,
        vleFluidType.nc,
        redirectorOutput) "Pointer to external medium memory";

    TILMedia.Internals.CriticalDataRecord crit "Critical data record" annotation (
       Placement(transformation(extent={{-80,60},{-60,80}}, rotation=0)));
    TILMedia.Internals.TransportPropertyRecord transp(eta(min=if
            computeTransportProperties then 0 else -1))
      "Transport property record" annotation (Placement(transformation(extent={{-80,
              -100},{-60,-80}}, rotation=0)));
    TILMedia.Internals.VLERecord VLE(final nc=vleFluidType.nc) annotation (
        Placement(transformation(extent={{-80,20},{-60,40}}, rotation=0)));
    TILMedia.Internals.AdditionalVLERecord VLEAdditional annotation (Placement(
          transformation(extent={{-80,-20},{-60,0}}, rotation=0)));
    TILMedia.Internals.VLETransportPropertyRecord VLETransp(eta_l(min=if
            computeVLETransportProperties then 0 else -1), eta_v(min=if
            computeVLETransportProperties then 0 else -1)) annotation (Placement(
          transformation(extent={{-80,-60},{-60,-40}}, rotation=0)));
  protected
    constant Real invalidValue=-1;
    final parameter Integer computeFlags=TILMedia.Internals.calcComputeFlags(
        computeTransportProperties,
        interpolateTransportProperties,
        computeSurfaceTension,
        deactivateTwoPhaseRegion);
    parameter Integer redirectorOutput=TILMedia.Internals.redirectModelicaFormatMessage();
  public
    function getProperties = TILMedia.Internals.getPropertiesVLE (
        d=d,
        h=h,
        p=p,
        s=s,
        T=T,
        cp=cp,
        q=q,
        d_l=VLE.d_l,
        h_l=VLE.h_l,
        p_l=VLE.p_l,
        s_l=VLE.s_l,
        T_l=VLE.T_l,
        d_v=VLE.d_v,
        h_v=VLE.h_v,
        p_v=VLE.p_v,
        s_v=VLE.s_v,
        T_v=VLE.T_v,
        d_crit=crit.d,
        h_crit=crit.h,
        p_crit=crit.p,
        s_crit=crit.s,
        T_crit=crit.T,
        Pr=transp.Pr,
        lambda=transp.lambda,
        eta=transp.eta,
        sigma=transp.sigma,
        Pr_l=VLETransp.Pr_l,
        Pr_v=VLETransp.Pr_v,
        lambda_l=VLETransp.lambda_l,
        lambda_v=VLETransp.lambda_v,
        eta_l=VLETransp.eta_l,
        eta_v=VLETransp.eta_v);

  equation
    M_i = TILMedia.VLEFluidObjectFunctions.molarMass_n(
          {i-1 for i in 1:vleFluidType.nc},vleFluidPointer);
    (crit.d, crit.h, crit.p, crit.s, crit.T) = TILMedia.Internals.VLEFluidObjectFunctions.cricondenbar_xi(xi,
      vleFluidPointer);
    //calculate molar mass
    M = 1/sum(cat(
      1,
      xi,
      {1 - sum(xi)}) ./ M_i);
    //calculate mole fraction
    xi = x .* M_i[1:end - 1]*(sum(cat(
      1,
      xi,
      {1 - sum(xi)}) ./ M_i));
    //xi = x.*M_i/M

    //Calculate Main Properties of state
    d = TILMedia.Internals.VLEFluidObjectFunctions.density_phxi(
      p,
      h,
      xi,
      vleFluidPointer);
    s = TILMedia.Internals.VLEFluidObjectFunctions.specificEntropy_phxi(
      p,
      h,
      xi,
      vleFluidPointer);
    T = TILMedia.Internals.VLEFluidObjectFunctions.temperature_phxi(
      p,
      h,
      xi,
      vleFluidPointer);

    //Calculate Additional Properties of state
    (q,cp,cv,beta,kappa,drhodp_hxi,drhodh_pxi,drhodxi_ph,w,gamma) =
      TILMedia.Internals.VLEFluidObjectFunctions.additionalProperties_phxi(
      p,
      h,
      xi,
      vleFluidPointer);

    //Calculate VLE Properties
    if (vleFluidType.nc == 1) then
      //VLE only depends on p or T
      (VLE.d_l,VLE.h_l,VLE.p_l,VLE.s_l,VLE.T_l,VLE.xi_l,VLE.d_v,VLE.h_v,VLE.p_v,
        VLE.s_v,VLE.T_v,VLE.xi_v) =
        TILMedia.Internals.VLEFluidObjectFunctions.VLEProperties_phxi(
        p,
        -1,
        zeros(0),
        vleFluidPointer);
    else
      //VLE of a mixture also depends on density/enthalpy/entropy/temperature
      (VLE.d_l,VLE.h_l,VLE.p_l,VLE.s_l,VLE.T_l,VLE.xi_l,VLE.d_v,VLE.h_v,VLE.p_v,
        VLE.s_v,VLE.T_v,VLE.xi_v) =
        TILMedia.Internals.VLEFluidObjectFunctions.VLEProperties_phxi(
        p,
        h,
        xi,
        vleFluidPointer);
    end if;

    //Calculate Transport Properties
    if computeTransportProperties then
      transp =
        TILMedia.Internals.VLEFluidObjectFunctions.transportPropertyRecord_phxi(
        p,
        h,
        xi,
        vleFluidPointer);
    else
      transp = TILMedia.Internals.TransportPropertyRecord(
        invalidValue,
        invalidValue,
        invalidValue,
        invalidValue);
    end if;

    //compute VLE Additional Properties
    if computeVLEAdditionalProperties then
      if (vleFluidType.nc == 1) then
        //VLE only depends on p or T
        (VLEAdditional.cp_l,VLEAdditional.beta_l,VLEAdditional.kappa_l,
          VLEAdditional.cp_v,VLEAdditional.beta_v,VLEAdditional.kappa_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLEAdditionalProperties_phxi(
          p,
          -1,
          zeros(vleFluidType.nc - 1),
          vleFluidPointer);
      else
        //VLE of a mixture also depends on density/enthalpy/entropy/temperature
        (VLEAdditional.cp_l,VLEAdditional.beta_l,VLEAdditional.kappa_l,
          VLEAdditional.cp_v,VLEAdditional.beta_v,VLEAdditional.kappa_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLEAdditionalProperties_phxi(
          p,
          h,
          xi,
          vleFluidPointer);
      end if;
    else
      VLEAdditional.cp_l = invalidValue;
      VLEAdditional.beta_l = invalidValue;
      VLEAdditional.kappa_l = invalidValue;
      VLEAdditional.cp_v = invalidValue;
      VLEAdditional.beta_v = invalidValue;
      VLEAdditional.kappa_v = invalidValue;
    end if;

    //compute VLE Transport Properties
    if computeVLETransportProperties then
      if (vleFluidType.nc == 1) then
        //VLE only depends on p or T
        (VLETransp.Pr_l,VLETransp.Pr_v,VLETransp.lambda_l,VLETransp.lambda_v,
          VLETransp.eta_l,VLETransp.eta_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLETransportPropertyRecord_phxi(
          p,
          -1,
          zeros(0),
          vleFluidPointer);
      else
        //VLE of a mixture also depends on density/enthalpy/entropy/temperature
        (VLETransp.Pr_l,VLETransp.Pr_v,VLETransp.lambda_l,VLETransp.lambda_v,
          VLETransp.eta_l,VLETransp.eta_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLETransportPropertyRecord_phxi(
          p,
          h,
          xi,
          vleFluidPointer);
      end if;
    else
      VLETransp.Pr_l = invalidValue;
      VLETransp.Pr_v = invalidValue;
      VLETransp.lambda_l = invalidValue;
      VLETransp.lambda_v = invalidValue;
      VLETransp.eta_l = invalidValue;
      VLETransp.eta_v = invalidValue;
    end if;

    /*
    Documentation(info="<html>
<p>
Standard refrigerant model that can be used in most applications. This model contains all three data records:
<b>CriticalDataRecord</b>, <b>SaturationPropertyRecord</b> and <b>TransportPropertyRecord</b>. <p> For ppf-media 
data transport properties are roughly estimated. For R245fa no transport properties are implemented yet.</p>
More information on this model can be found in the
<a href=\"Modelica:TILMedia.UsersGuide\">Users Guide</a>.
</p>
</html>")*/
    annotation (
      Placement(transformation(extent={{-80,20},{-60,40}}, rotation=0)),
      defaultComponentName="vleFluid",
      Icon(graphics={Text(
            extent={{-120,-60},{120,-100}},
            lineColor={153,204,0},
            textString="%name"), Bitmap(
            extent={{-100,-100},{100,100}},
            imageSource=
                "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",
            fileName="modelica://TILMedia/Images/VLE_ph.png")}),
      Documentation(info="<html>
                   <p>
                   The VLE-fluid model VLEFluid_ph calculates the thermopyhsical property data with given inputs: pressure (p), enthalpy (h), massfraction (xi) and the parameter vleFluidType.<br>
                   The interface and the way of using, is demonstrated in the Testers -> <a href=\"Modelica:TILMedia.Testers.TestVLEFluid\">TestVLEFluid</a>.
                   </p>
                   <hr>
                   </html>"));
  end VLEFluid_ph;

  model VLEFluid_ps
    "VLE-Fluid model describing super-critical, subcooled, superheated fluid including the vapour liquid equilibrium (p, s and xi as independent variables)"
    replaceable parameter TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType
      constrainedby TILMedia.VLEFluidTypes.BaseVLEFluid
      "type record of the VLE fluid or VLE fluid mixture"
      annotation (choicesAllMatching=true);

    parameter Boolean stateSelectPreferForInputs=false
      "=true, StateSelect.prefer is set for input variables"
      annotation (Evaluate=true, Dialog(tab="Advanced", group "StateSelect"));
    parameter Boolean computeTransportProperties=false
      "=true, if transport properties are calculated";
    parameter Boolean interpolateTransportProperties=true
      "Interpolate transport properties in vapor dome"
      annotation (Dialog(tab="Advanced"));
    parameter Boolean computeSurfaceTension=true
      annotation (Dialog(tab="Advanced"));
    parameter Boolean computeVLEAdditionalProperties=false
      "Compute detailed vapour liquid equilibrium properties"
      annotation (Evaluate=true);
    parameter Boolean computeVLETransportProperties=false
      "Compute detailed vapour liquid equilibrium transport properties"
      annotation (Evaluate=true);
    parameter Boolean deactivateTwoPhaseRegion=false
      "Deactivate calculation of two phase region"
      annotation (Evaluate=true);

    //Base Properties
    Modelica.SIunits.Density d "Density";
    Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
    input Modelica.SIunits.AbsolutePressure p(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Pressure" annotation(Dialog);
    input Modelica.SIunits.SpecificEntropy s(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Specific entropy" annotation(Dialog);
    Modelica.SIunits.Temperature T "Temperature";
    input Modelica.SIunits.MassFraction[vleFluidType.nc - 1] xi(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)=vleFluidType.xi_default
      "Mass Fraction of Component i" annotation(Dialog);
    SI.MoleFraction x[vleFluidType.nc - 1] "Mole fraction";
    SI.MolarMass M "Average molar mass";

    //Additional Properties
    SI.MassFraction q "Steam mass fraction (quality)";
    SI.SpecificHeatCapacity cp "Specific isobaric heat capacity cp";
    SI.SpecificHeatCapacity cv "Specific isochoric heat capacity cv";
    SI.LinearExpansionCoefficient beta "Isobaric thermal expansion coefficient";
    SI.Compressibility kappa "Isothermal compressibility";
    SI.Velocity w "Speed of sound";
    SI.DerDensityByEnthalpy drhodh_pxi
      "1st derivative of density wrt specific enthalpy at constant pressure and mass fraction";
    SI.DerDensityByPressure drhodp_hxi
      "1st derivative of density wrt pressure at specific enthalpy and mass fraction";
    TILMedia.Internals.Units.DensityDerMassFraction drhodxi_ph[vleFluidType.nc - 1]
      "1st derivative of density wrt mass fraction of water at constant pressure and specific enthalpy";
    Real gamma "Heat capacity ratio aka isentropic expansion factor";

    SI.MolarMass M_i[vleFluidType.nc] "Molar mass of component i";

    TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer=
        TILMedia.VLEFluidObjectFunctions.VLEFluidPointer(
        vleFluidType.concatVLEFluidName,
        computeFlags,
        vleFluidType.mixingRatio_propertyCalculation[1:end-1]/sum(vleFluidType.mixingRatio_propertyCalculation),
        vleFluidType.nc_propertyCalculation,
        vleFluidType.nc,
        redirectorOutput) "Pointer to external medium memory";

    TILMedia.Internals.CriticalDataRecord crit "Critical data record" annotation (
       Placement(transformation(extent={{-80,60},{-60,80}}, rotation=0)));
    TILMedia.Internals.TransportPropertyRecord transp(eta(min=if
            computeTransportProperties then 0 else -1))
      "Transport property record" annotation (Placement(transformation(extent={{-80,
              -100},{-60,-80}}, rotation=0)));
    TILMedia.Internals.VLERecord VLE(final nc=vleFluidType.nc) annotation (
        Placement(transformation(extent={{-80,20},{-60,40}}, rotation=0)));
    TILMedia.Internals.AdditionalVLERecord VLEAdditional annotation (Placement(
          transformation(extent={{-80,-20},{-60,0}}, rotation=0)));
    TILMedia.Internals.VLETransportPropertyRecord VLETransp(eta_l(min=if
            computeVLETransportProperties then 0 else -1), eta_v(min=if
            computeVLETransportProperties then 0 else -1)) annotation (Placement(
          transformation(extent={{-80,-60},{-60,-40}}, rotation=0)));
  protected
    constant Real invalidValue=-1;
    final parameter Integer computeFlags=TILMedia.Internals.calcComputeFlags(
        computeTransportProperties,
        interpolateTransportProperties,
        computeSurfaceTension,
        deactivateTwoPhaseRegion);
    parameter Integer redirectorOutput=TILMedia.Internals.redirectModelicaFormatMessage();
  public
    function getProperties = TILMedia.Internals.getPropertiesVLE (
        d=d,
        h=h,
        p=p,
        s=s,
        T=T,
        cp=cp,
        q=q,
        d_l=VLE.d_l,
        h_l=VLE.h_l,
        p_l=VLE.p_l,
        s_l=VLE.s_l,
        T_l=VLE.T_l,
        d_v=VLE.d_v,
        h_v=VLE.h_v,
        p_v=VLE.p_v,
        s_v=VLE.s_v,
        T_v=VLE.T_v,
        d_crit=crit.d,
        h_crit=crit.h,
        p_crit=crit.p,
        s_crit=crit.s,
        T_crit=crit.T,
        Pr=transp.Pr,
        lambda=transp.lambda,
        eta=transp.eta,
        sigma=transp.sigma,
        Pr_l=VLETransp.Pr_l,
        Pr_v=VLETransp.Pr_v,
        lambda_l=VLETransp.lambda_l,
        lambda_v=VLETransp.lambda_v,
        eta_l=VLETransp.eta_l,
        eta_v=VLETransp.eta_v);

  equation
    M_i = TILMedia.VLEFluidObjectFunctions.molarMass_n(
          {i-1 for i in 1:vleFluidType.nc},vleFluidPointer);
    (crit.d, crit.h, crit.p, crit.s, crit.T) = TILMedia.Internals.VLEFluidObjectFunctions.cricondenbar_xi(xi,
      vleFluidPointer);
    //calculate molar mass
    M = 1/sum(cat(
      1,
      xi,
      {1 - sum(xi)}) ./ M_i);
    //calculate mole fraction
    xi = x .* M_i[1:end - 1]*(sum(cat(
      1,
      xi,
      {1 - sum(xi)}) ./ M_i));
    //xi = x.*M_i/M

    //Calculate Main Properties of state
    d = TILMedia.Internals.VLEFluidObjectFunctions.density_psxi(
      p,
      s,
      xi,
      vleFluidPointer);
    h = TILMedia.Internals.VLEFluidObjectFunctions.specificEnthalpy_psxi(
      p,
      s,
      xi,
      vleFluidPointer);
    T = TILMedia.Internals.VLEFluidObjectFunctions.temperature_psxi(
      p,
      s,
      xi,
      vleFluidPointer);

    //Calculate Additional Properties of state
    (q,cp,cv,beta,kappa,drhodp_hxi,drhodh_pxi,drhodxi_ph,w,gamma) =
      TILMedia.Internals.VLEFluidObjectFunctions.additionalProperties_phxi(
      p,
      h,
      xi,
      vleFluidPointer);

    //Calculate VLE Properties
    if (vleFluidType.nc == 1) then
      //VLE only depends on p or T
      (VLE.d_l,VLE.h_l,VLE.p_l,VLE.s_l,VLE.T_l,VLE.xi_l,VLE.d_v,VLE.h_v,VLE.p_v,
        VLE.s_v,VLE.T_v,VLE.xi_v) =
        TILMedia.Internals.VLEFluidObjectFunctions.VLEProperties_phxi(
        p,
        -1,
        zeros(0),
        vleFluidPointer);
    else
      //VLE of a mixture also depends on density/enthalpy/entropy/temperature
      (VLE.d_l,VLE.h_l,VLE.p_l,VLE.s_l,VLE.T_l,VLE.xi_l,VLE.d_v,VLE.h_v,VLE.p_v,
        VLE.s_v,VLE.T_v,VLE.xi_v) =
        TILMedia.Internals.VLEFluidObjectFunctions.VLEProperties_phxi(
        p,
        h,
        xi,
        vleFluidPointer);
    end if;

    //Calculate Transport Properties
    if computeTransportProperties then
      transp =
        TILMedia.Internals.VLEFluidObjectFunctions.transportPropertyRecord_phxi(
        p,
        h,
        xi,
        vleFluidPointer);
    else
      transp = TILMedia.Internals.TransportPropertyRecord(
        invalidValue,
        invalidValue,
        invalidValue,
        invalidValue);
    end if;

    //compute VLE Additional Properties
    if computeVLEAdditionalProperties then
      if (vleFluidType.nc == 1) then
        //VLE only depends on p or T
        (VLEAdditional.cp_l,VLEAdditional.beta_l,VLEAdditional.kappa_l,
          VLEAdditional.cp_v,VLEAdditional.beta_v,VLEAdditional.kappa_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLEAdditionalProperties_phxi(
          p,
          -1,
          zeros(vleFluidType.nc - 1),
          vleFluidPointer);
      else
        //VLE of a mixture also depends on density/enthalpy/entropy/temperature
        (VLEAdditional.cp_l,VLEAdditional.beta_l,VLEAdditional.kappa_l,
          VLEAdditional.cp_v,VLEAdditional.beta_v,VLEAdditional.kappa_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLEAdditionalProperties_phxi(
          p,
          h,
          xi,
          vleFluidPointer);
      end if;
    else
      VLEAdditional.cp_l = invalidValue;
      VLEAdditional.beta_l = invalidValue;
      VLEAdditional.kappa_l = invalidValue;
      VLEAdditional.cp_v = invalidValue;
      VLEAdditional.beta_v = invalidValue;
      VLEAdditional.kappa_v = invalidValue;
    end if;

    //compute VLE Transport Properties
    if computeVLETransportProperties then
      if (vleFluidType.nc == 1) then
        //VLE only depends on p or T
        (VLETransp.Pr_l,VLETransp.Pr_v,VLETransp.lambda_l,VLETransp.lambda_v,
          VLETransp.eta_l,VLETransp.eta_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLETransportPropertyRecord_phxi(
          p,
          -1,
          zeros(0),
          vleFluidPointer);
      else
        //VLE of a mixture also depends on density/enthalpy/entropy/temperature
        (VLETransp.Pr_l,VLETransp.Pr_v,VLETransp.lambda_l,VLETransp.lambda_v,
          VLETransp.eta_l,VLETransp.eta_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLETransportPropertyRecord_phxi(
          p,
          h,
          xi,
          vleFluidPointer);
      end if;
    else
      VLETransp.Pr_l = invalidValue;
      VLETransp.Pr_v = invalidValue;
      VLETransp.lambda_l = invalidValue;
      VLETransp.lambda_v = invalidValue;
      VLETransp.eta_l = invalidValue;
      VLETransp.eta_v = invalidValue;
    end if;

    /*    Documentation(info="<html>
<p>
Standard refrigerant model that can be used in most applications. This model contains all three data records:
<b>CriticalDataRecord</b>, <b>SaturationPropertyRecord</b> and <b>TransportPropertyRecord</b>. <p> For ppf-media 
data transport properties are roughly estimated. For R245fa no transport properties are implemented yet.</p>
More information on this model can be found in the
<a href=\"Modelica:TILMedia.UsersGuide\">Users Guide</a>.
</p>
</html>"),*/
    annotation (
      defaultComponentName="vleFluid",
      Icon(graphics={Text(
            extent={{-120,-60},{120,-100}},
            lineColor={153,204,0},
            textString="%name"), Bitmap(
            extent={{-100,-100},{100,100}},
            imageSource=
                "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",
            fileName="modelica://TILMedia/Images/VLE_ps.png")}),
      Documentation(info="<html>
                   <p>
                   The VLE-fluid model VLEFluid_ps calculates the thermopyhsical property data with given inputs: pressure (p), entropy (s, massfraction (xi)) and the parameter vleFluidType.<br>
                   The interface and the way of using, is demonstrated in the Testers -> <a href=\"Modelica:TILMedia.Testers.TestVLEFluid\">TestVLEFluid</a>.
                   </p>
                   <hr>
                   </html>"));
  end VLEFluid_ps;

  model VLEFluid_pT
    "VLE-Fluid model describing super-critical, subcooled, superheated fluid including the vapour liquid equilibrium (p, T and xi as independent variables)"
    replaceable parameter TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType
      constrainedby TILMedia.VLEFluidTypes.BaseVLEFluid
      "type record of the VLE fluid or VLE fluid mixture"
      annotation (choicesAllMatching=true);

    parameter Boolean stateSelectPreferForInputs=false
      "=true, StateSelect.prefer is set for input variables"
      annotation (Evaluate=true, Dialog(tab="Advanced", group "StateSelect"));
    parameter Boolean computeTransportProperties=false
      "=true, if transport properties are calculated";
    parameter Boolean interpolateTransportProperties=true
      "Interpolate transport properties in vapor dome"
      annotation (Dialog(tab="Advanced"));
    parameter Boolean computeSurfaceTension=true
      annotation (Dialog(tab="Advanced"));
    parameter Boolean computeVLEAdditionalProperties=false
      "Compute detailed vapour liquid equilibrium properties"
      annotation (Evaluate=true);
    parameter Boolean computeVLETransportProperties=false
      "Compute detailed vapour liquid equilibrium transport properties"
      annotation (Evaluate=true);
    parameter Boolean deactivateTwoPhaseRegion=false
      "Deactivate calculation of two phase region"
      annotation (Evaluate=true);

    //Base Properties
    Modelica.SIunits.Density d "Density";
    Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
    input Modelica.SIunits.AbsolutePressure p(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Pressure" annotation(Dialog);
    Modelica.SIunits.SpecificEntropy s "Specific entropy";
    input Modelica.SIunits.Temperature T(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)
      "Temperature" annotation(Dialog);
    input Modelica.SIunits.MassFraction[vleFluidType.nc - 1] xi(stateSelect=if (
          stateSelectPreferForInputs) then StateSelect.prefer else StateSelect.default)=vleFluidType.xi_default
      "Mass Fraction of Component i" annotation(Dialog);
    SI.MoleFraction x[vleFluidType.nc - 1] "Mole fraction";
    SI.MolarMass M "Average molar mass";

    //Additional Properties
    SI.MassFraction q "Steam mass fraction (quality)";
    SI.SpecificHeatCapacity cp "Specific isobaric heat capacity cp";
    SI.SpecificHeatCapacity cv "Specific isochoric heat capacity cv";
    SI.LinearExpansionCoefficient beta "Isobaric thermal expansion coefficient";
    SI.Compressibility kappa "Isothermal compressibility";
    SI.Velocity w "Speed of sound";
    SI.DerDensityByEnthalpy drhodh_pxi
      "1st derivative of density wrt specific enthalpy at constant pressure and mass fraction";
    SI.DerDensityByPressure drhodp_hxi
      "1st derivative of density wrt pressure at specific enthalpy and mass fraction";
    TILMedia.Internals.Units.DensityDerMassFraction drhodxi_ph[vleFluidType.nc - 1]
      "1st derivative of density wrt mass fraction of water at constant pressure and specific enthalpy";
    Real gamma "Heat capacity ratio aka isentropic expansion factor";

    SI.MolarMass M_i[vleFluidType.nc] "Molar mass of component i";

    TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer=
        TILMedia.VLEFluidObjectFunctions.VLEFluidPointer(
        vleFluidType.concatVLEFluidName,
        computeFlags,
        vleFluidType.mixingRatio_propertyCalculation[1:end-1]/sum(vleFluidType.mixingRatio_propertyCalculation),
        vleFluidType.nc_propertyCalculation,
        vleFluidType.nc,
        redirectorOutput) "Pointer to external medium memory";

    TILMedia.Internals.CriticalDataRecord crit "Critical data record" annotation (
       Placement(transformation(extent={{-80,60},{-60,80}}, rotation=0)));
    TILMedia.Internals.TransportPropertyRecord transp(eta(min=if
            computeTransportProperties then 0 else -1))
      "Transport property record" annotation (Placement(transformation(extent={{-80,
              -100},{-60,-80}}, rotation=0)));
    TILMedia.Internals.VLERecord VLE(final nc=vleFluidType.nc) annotation (
        Placement(transformation(extent={{-80,20},{-60,40}}, rotation=0)));
    TILMedia.Internals.AdditionalVLERecord VLEAdditional annotation (Placement(
          transformation(extent={{-80,-20},{-60,0}}, rotation=0)));
    TILMedia.Internals.VLETransportPropertyRecord VLETransp(eta_l(min=if
            computeVLETransportProperties then 0 else -1), eta_v(min=if
            computeVLETransportProperties then 0 else -1)) annotation (Placement(
          transformation(extent={{-80,-60},{-60,-40}}, rotation=0)));
  protected
    constant Real invalidValue=-1;
    final parameter Integer computeFlags=TILMedia.Internals.calcComputeFlags(
        computeTransportProperties,
        interpolateTransportProperties,
        computeSurfaceTension,
        deactivateTwoPhaseRegion);
    parameter Integer redirectorOutput=TILMedia.Internals.redirectModelicaFormatMessage();
  public
    function getProperties = TILMedia.Internals.getPropertiesVLE (
        d=d,
        h=h,
        p=p,
        s=s,
        T=T,
        cp=cp,
        q=q,
        d_l=VLE.d_l,
        h_l=VLE.h_l,
        p_l=VLE.p_l,
        s_l=VLE.s_l,
        T_l=VLE.T_l,
        d_v=VLE.d_v,
        h_v=VLE.h_v,
        p_v=VLE.p_v,
        s_v=VLE.s_v,
        T_v=VLE.T_v,
        d_crit=crit.d,
        h_crit=crit.h,
        p_crit=crit.p,
        s_crit=crit.s,
        T_crit=crit.T,
        Pr=transp.Pr,
        lambda=transp.lambda,
        eta=transp.eta,
        sigma=transp.sigma,
        Pr_l=VLETransp.Pr_l,
        Pr_v=VLETransp.Pr_v,
        lambda_l=VLETransp.lambda_l,
        lambda_v=VLETransp.lambda_v,
        eta_l=VLETransp.eta_l,
        eta_v=VLETransp.eta_v);
  equation
    M_i = TILMedia.VLEFluidObjectFunctions.molarMass_n(
          {i-1 for i in 1:vleFluidType.nc},vleFluidPointer);
    (crit.d, crit.h, crit.p, crit.s, crit.T) = TILMedia.Internals.VLEFluidObjectFunctions.cricondenbar_xi(xi,
      vleFluidPointer);
    //calculate molar mass
    M = 1/sum(cat(
      1,
      xi,
      {1 - sum(xi)}) ./ M_i);
    //calculate mole fraction
    xi = x .* M_i[1:end - 1]*(sum(cat(
      1,
      xi,
      {1 - sum(xi)}) ./ M_i));
    //xi = x.*M_i/M

    //Calculate Main Properties of state
    d = TILMedia.Internals.VLEFluidObjectFunctions.density_pTxi(
      p,
      T,
      xi,
      vleFluidPointer);
    h = TILMedia.Internals.VLEFluidObjectFunctions.specificEnthalpy_pTxi(
      p,
      T,
      xi,
      vleFluidPointer);
    s = TILMedia.Internals.VLEFluidObjectFunctions.specificEntropy_pTxi(
      p,
      T,
      xi,
      vleFluidPointer);

    //Calculate Additional Properties of state
    (q,cp,cv,beta,kappa,drhodp_hxi,drhodh_pxi,drhodxi_ph,w,gamma) =
      TILMedia.Internals.VLEFluidObjectFunctions.additionalProperties_phxi(
      p,
      h,
      xi,
      vleFluidPointer);

    //Calculate VLE Properties
    if (vleFluidType.nc == 1) then
      //VLE only depends on p or T
      (VLE.d_l,VLE.h_l,VLE.p_l,VLE.s_l,VLE.T_l,VLE.xi_l,VLE.d_v,VLE.h_v,VLE.p_v,
        VLE.s_v,VLE.T_v,VLE.xi_v) =
        TILMedia.Internals.VLEFluidObjectFunctions.VLEProperties_phxi(
        p,
        -1,
        zeros(0),
        vleFluidPointer);
    else
      //VLE of a mixture also depends on density/enthalpy/entropy/temperature
      (VLE.d_l,VLE.h_l,VLE.p_l,VLE.s_l,VLE.T_l,VLE.xi_l,VLE.d_v,VLE.h_v,VLE.p_v,
        VLE.s_v,VLE.T_v,VLE.xi_v) =
        TILMedia.Internals.VLEFluidObjectFunctions.VLEProperties_phxi(
        p,
        h,
        xi,
        vleFluidPointer);
    end if;

    //Calculate Transport Properties
    if computeTransportProperties then
      transp =
        TILMedia.Internals.VLEFluidObjectFunctions.transportPropertyRecord_phxi(
        p,
        h,
        xi,
        vleFluidPointer);
    else
      transp = TILMedia.Internals.TransportPropertyRecord(
        invalidValue,
        invalidValue,
        invalidValue,
        invalidValue);
    end if;

    //compute VLE Additional Properties
    if computeVLEAdditionalProperties then
      if (vleFluidType.nc == 1) then
        //VLE only depends on p or T
        (VLEAdditional.cp_l,VLEAdditional.beta_l,VLEAdditional.kappa_l,
          VLEAdditional.cp_v,VLEAdditional.beta_v,VLEAdditional.kappa_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLEAdditionalProperties_phxi(
          p,
          -1,
          zeros(vleFluidType.nc - 1),
          vleFluidPointer);
      else
        //VLE of a mixture also depends on density/enthalpy/entropy/temperature
        (VLEAdditional.cp_l,VLEAdditional.beta_l,VLEAdditional.kappa_l,
          VLEAdditional.cp_v,VLEAdditional.beta_v,VLEAdditional.kappa_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLEAdditionalProperties_phxi(
          p,
          h,
          xi,
          vleFluidPointer);
      end if;
    else
      VLEAdditional.cp_l = invalidValue;
      VLEAdditional.beta_l = invalidValue;
      VLEAdditional.kappa_l = invalidValue;
      VLEAdditional.cp_v = invalidValue;
      VLEAdditional.beta_v = invalidValue;
      VLEAdditional.kappa_v = invalidValue;
    end if;

    //compute VLE Transport Properties
    if computeVLETransportProperties then
      if (vleFluidType.nc == 1) then
        //VLE only depends on p or T
        (VLETransp.Pr_l,VLETransp.Pr_v,VLETransp.lambda_l,VLETransp.lambda_v,
          VLETransp.eta_l,VLETransp.eta_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLETransportPropertyRecord_phxi(
          p,
          -1,
          zeros(0),
          vleFluidPointer);
      else
        //VLE of a mixture also depends on density/enthalpy/entropy/temperature
        (VLETransp.Pr_l,VLETransp.Pr_v,VLETransp.lambda_l,VLETransp.lambda_v,
          VLETransp.eta_l,VLETransp.eta_v) =
          TILMedia.Internals.VLEFluidObjectFunctions.VLETransportPropertyRecord_phxi(
          p,
          h,
          xi,
          vleFluidPointer);
      end if;
    else
      VLETransp.Pr_l = invalidValue;
      VLETransp.Pr_v = invalidValue;
      VLETransp.lambda_l = invalidValue;
      VLETransp.lambda_v = invalidValue;
      VLETransp.eta_l = invalidValue;
      VLETransp.eta_v = invalidValue;
    end if;

    /*Documentation(info="<html>
<p>
Standard refrigerant model that can be used in most applications. This model contains all three data records:
<b>CriticalDataRecord</b>, <b>SaturationPropertyRecord</b> and <b>TransportPropertyRecord</b>. <p> For ppf-media 
data transport properties are roughly estimated. For R245fa no transport properties are implemented yet.</p>
More information on this model can be found in the
<a href=\"Modelica:TILMedia.UsersGuide\">Users Guide</a>.
</p>
</html>"),*/
    annotation (
      defaultComponentName="vleFluid",
      Icon(graphics={Text(
            extent={{-120,-60},{120,-100}},
            lineColor={153,204,0},
            textString="%name"), Bitmap(
            extent={{-100,-100},{100,100}},
            imageSource=
                "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",
            fileName="modelica://TILMedia/Images/VLE_pT.png")}),
      Documentation(info="<html>
                   <p>
                   The VLE-fluid model VLEFluid_pT calculates the thermopyhsical property data with given inputs: pressure (p), temperature (T), massfraction (xi) and the parameter vleFluidType.<br>
                   The interface and the way of using, is demonstrated in the Testers -> <a href=\"Modelica:TILMedia.Testers.TestVLEFluid\">TestVLEFluid</a>.
                   </p>
                   <hr>
                   </html>"));
  end VLEFluid_pT;

  package GasFunctions
  "Package for calculation of gas vapor properties with a functional call"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

  function density_pTxi
  // Don't use these functions during simulation, Medium classes are always faster! Use only for start and initial values.
    input TILMedia.GasTypes.BaseGas gasType "Gas type" annotation(choicesAllMatching=true);
    input SI.AbsolutePressure p "Pressure";
    input SI.Temperature T "Temperature";
    input SI.MassFraction[:] xi=zeros(gasType.nc-1)
        "Mass fractions of the first nc-1 components";
    output SI.Density d "Density";
  algorithm
    d := TILMedia.Internals.GasFunctions.density_pTxi(p,T,xi,gasType.concatGasName, gasType.nc+TILMedia.Internals.redirectModelicaFormatMessage(), gasType.condensingIndex);
    annotation(Inline=true, Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/Gas_Function.png")}));
  end density_pTxi;

  function specificEnthalpy_pTxi
  // Don't use these functions during simulation, Medium classes are always faster! Use only for start and initial values.
    input TILMedia.GasTypes.BaseGas gasType "Gas type" annotation(choicesAllMatching=true);
    input SI.AbsolutePressure p "Pressure";
    input SI.Temperature T "Temperature";
    input SI.MassFraction[:] xi=zeros(gasType.nc-1)
        "Mass fractions of the first nc-1 components";
    output SI.SpecificEnthalpy h "Specific enthalpy";
  algorithm
    h := TILMedia.Internals.GasFunctions.specificEnthalpy_pTxi(p,T,xi,gasType.concatGasName, gasType.nc+TILMedia.Internals.redirectModelicaFormatMessage(), gasType.condensingIndex);
    annotation(Inline=true, Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/Gas_Function.png")}));
  end specificEnthalpy_pTxi;
  end GasFunctions;

  package GasObjectFunctions
  "Package for calculation of gas vapor properties with a functional call, referencing existing external objects for highspeed evaluation"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    class GasPointer
       extends ExternalObject;
       function constructor "get memory"
        input String mediumName;
        input Integer flags;
        input Real[:] xi;
        input Integer nc_propertyCalculation;
        input Integer nc;
        input Integer condensingIndex;
        input Integer redirectorDummy;
        output GasPointer gasPointer;
        external "C" gasPointer = TILMedia_Gas_createExternalObject(mediumName, flags, xi, nc_propertyCalculation, nc, condensingIndex)
                                             annotation(Library="TILMedia341");
       end constructor;

       function destructor "free memory"
        input GasPointer gasPointer;
        external "C" TILMedia_Gas_destroyExternalObject(gasPointer)
                                             annotation(Library="TILMedia341");
       end destructor;
    end GasPointer;

  function molarMass_n
    input Integer compNo "Component ID";
    input TILMedia.GasObjectFunctions.GasPointer gasPointer;
    output SI.MolarMass M_i "Molar mass of component i";
  external "C" M_i = TILMedia_GasObjectFunctions_molarMass_n(compNo, gasPointer)
    annotation(Library="TILMedia341");
    annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/Gas_Function.png")}));
  end molarMass_n;
  end GasObjectFunctions;

  package GasTypes
  "Gases and Gas Vapor mixtures, that can be used or composed in TILMedia"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    record BaseGas "Base record for gas definitions"
      extends Internals.ClassTypes.Record;
      constant Boolean fixedMixingRatio
        "Treat medium as pseudo pure in Modelica if it is a mixture" annotation(HideResult = true);
      constant Integer nc_propertyCalculation(min=1)
        "Number of components for fluid property calculations" annotation(HideResult = true);
      final constant Integer nc=if fixedMixingRatio then 1 else nc_propertyCalculation
        "Number of components in Modelica models"
                                                annotation(Evaluate=true, HideResult = true);
      constant Internals.GasName[nc_propertyCalculation] gasNames
        "Array of gas names"                             annotation(choices);
      constant Real[nc_propertyCalculation] mixingRatio_propertyCalculation
        "Mixing ratio for fluid property calculation (={1} for pure components)" annotation(HideResult = true);
      constant Real[nc] defaultMixingRatio = if fixedMixingRatio then {1} else mixingRatio_propertyCalculation
        "Default composition for models in Modelica (={1} for pure components)" annotation(HideResult = true);
      constant Real xi_default[nc-1] = defaultMixingRatio[1:end-1]/sum(defaultMixingRatio)
        "Default mass fractions" annotation(HideResult = true);
      constant Integer condensingIndex
        "Index of condensing component (=0, if no condensation is desired)"
        annotation(HideResult = true);
      constant String concatGasName=TILMedia.Internals.concatNames(gasNames);
      constant Integer ID=0
        "ID is used to map the selected Gas to the sim.cumulatedGasMass array item" annotation(HideResult = true);
    end BaseGas;

    record TILMedia_MoistAir "TILMedia.MoistAir"
      extends TILMedia.GasTypes.BaseGas(
        final fixedMixingRatio=false,
        final nc_propertyCalculation=2,
        final gasNames={"",""},
        final concatGasName="TILMedia.MoistAir",
        final mixingRatio_propertyCalculation={0.0001,1},
        final condensingIndex=1);
    end TILMedia_MoistAir;

    record VDI4670_MoistAir "VDI4670.MoistAir"
      extends TILMedia.GasTypes.BaseGas(
        final fixedMixingRatio=false,
        final nc_propertyCalculation=2,
        final gasNames={"",""},
        final concatGasName="VDI4670.MoistAir",
        final mixingRatio_propertyCalculation={0.0001,1},
        final condensingIndex=1);
    end VDI4670_MoistAir;

    record VDIWA_MoistAir_nc3 "Detailed moist air using VDIWA 2006"
      extends TILMedia.GasTypes.BaseGas(
        final fixedMixingRatio=false,
        final nc_propertyCalculation=3,
        final gasNames={"VDIWA2006.Water","VDIWA2006.Nitrogen","VDIWA2006.Oxygen"},
        final condensingIndex=1,
        final mixingRatio_propertyCalculation={0.001,0.7,0.3});

    end VDIWA_MoistAir_nc3;
  end GasTypes;

  package LiquidTypes "Liquid types, that can be used in TILMedia"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    record BaseLiquid "Base record for liquid definitions"
      extends Internals.ClassTypes.Record;
      constant Boolean fixedMixingRatio
        "Treat medium as pseudo pure in Modelica if it is a mixture" annotation(HideResult = true);
      constant Integer nc_propertyCalculation(min=1)
        "Number of components for fluid property calculations" annotation(HideResult = true);
      final constant Integer nc=if fixedMixingRatio then 1 else nc_propertyCalculation
        "Number of components in Modelica models" annotation(Evaluate=true, HideResult = true);
      constant Internals.LiquidName[nc] liquidNames "Array of liquid names"
                                                                  annotation(choices);
      constant Real[nc_propertyCalculation] mixingRatio_propertyCalculation
        "Mixing ratio for fluid property calculation (={1} for pure components)" annotation(HideResult = true);
      constant Real[nc] defaultMixingRatio = if fixedMixingRatio then {1} else mixingRatio_propertyCalculation
        "Default composition for models in Modelica (={1} for pure components)" annotation(HideResult = true);
      constant Real xi_default[nc-1] = defaultMixingRatio[1:end-1]/sum(defaultMixingRatio)
        "Default mass fractions" annotation(HideResult = true);
      constant String concatLiquidName=TILMedia.Internals.concatNames(liquidNames);
      constant Integer ID=0
        "ID is used to map the selected Liquid to the sim.cumulatedLiquidMass array item" annotation(HideResult = true);
    end BaseLiquid;

    record TILMedia_Water "TILMedia.Water"
      extends TILMedia.LiquidTypes.BaseLiquid(
        final fixedMixingRatio=false,
        final nc_propertyCalculation=1,
        final liquidNames={""},
        final mixingRatio_propertyCalculation={1},
        final concatLiquidName="TILMedia.Water");
    end TILMedia_Water;

    record TILMedia_Glysantin_60 "TILMedia.Glysantin_60 % mass fraction"
      extends TILMedia.LiquidTypes.BaseLiquid(
        final fixedMixingRatio=false,
        final nc_propertyCalculation=1,
        final liquidNames={""},
        final mixingRatio_propertyCalculation={1},
        final concatLiquidName="TILMedia.Glysantin_60");
    end TILMedia_Glysantin_60;
  end LiquidTypes;

  package SLEMediumTypes "SLE Medium types that can be used in TILMedia"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    record BaseSLEMedium "Base record for solid definitions"
      extends Internals.ClassTypes.Record;
      constant Internals.SLEMediumName sleMediumName "SLE Medium name";
    end BaseSLEMedium;

    record TILMedia_AdBlue "TILMedia.AdBlue"
      extends TILMedia.SLEMediumTypes.BaseSLEMedium(
        sleMediumName="TILMedia.AdBlue");
    end TILMedia_AdBlue;
  end SLEMediumTypes;

  package SolidTypes "Solid types that can be used in TILMedia"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    partial model BaseSolid "Base model for solid definitions"
      constant SI.SpecificHeatCapacity cp_nominal
        "Specific heat capacity at standard reference point";
      constant SI.ThermalConductivity lambda_nominal
        "Thermal conductivity at standard reference point";

      constant SI.Density d "Density";
      input SI.Temperature T "Temperature";
      SI.SpecificHeatCapacity cp "Heat capacity";
      SI.ThermalConductivity lambda "Thermal conductivity";
    end BaseSolid;

    model TILMedia_Aluminum "TILMedia.Aluminum"
      extends TILMedia.SolidTypes.BaseSolid(
        final d = 2700.0,
        final cp_nominal = 920.0,
        final lambda_nominal = 215.0);
    equation
      cp=cp_nominal;
      lambda=lambda_nominal;
    end TILMedia_Aluminum;

    model TILMedia_Copper "TILMedia.Copper"
      extends TILMedia.SolidTypes.BaseSolid(
        final d = 8960.0,
        final cp_nominal = 380.0,
        final lambda_nominal = 298.0);
    equation
      cp=cp_nominal;
      lambda=lambda_nominal;
    end TILMedia_Copper;

    model TILMedia_Steel "TILMedia.Steel"
      extends TILMedia.SolidTypes.BaseSolid(
        final d = 7800.0,
        final cp_nominal = 490.0,
        final lambda_nominal = 40.0);
    equation
      cp=cp_nominal;
      lambda=lambda_nominal;
    end TILMedia_Steel;
  end SolidTypes;

  package VLEFluidFunctions
  "Package for calculation of VLEFluid properties with a functional call"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    function specificEnthalpy_pTxi
    // Don't use these functions during simulation, Medium classes are always faster! Only use for start and initial values.
      input TILMedia.VLEFluidTypes.BaseVLEFluid vleFluidType "VLEFluid type" annotation(choicesAllMatching=true);
      input SI.AbsolutePressure p "Pressure";
      input SI.Temperature T "Temperature";
      input SI.MassFraction[:] xi=zeros(vleFluidType.nc-1)
        "Mass fractions of the first nc-1 components";
      output SI.SpecificEnthalpy h "Specific enthalpy";
    algorithm
      h := TILMedia.Internals.VLEFluidFunctions.specificEnthalpy_pTxi(p,T,xi,vleFluidType.concatVLEFluidName, vleFluidType.nc+TILMedia.Internals.redirectModelicaFormatMessage());
      annotation(Inline=true, Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/VLE_Function.png")}));
    end specificEnthalpy_pTxi;
  end VLEFluidFunctions;

  package VLEFluidObjectFunctions
  "Package for calculation of VLEFLuid properties with a functional call, referencing existing external objects for highspeed evaluation"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

    class VLEFluidPointer
       extends ExternalObject;
       function constructor "get memory"
        input String vleFluidName;
        input Integer flags;
        input Real[:] xi;
        input Integer nc_propertyCalculation;
        input Integer nc;
        input Integer redirectorDummy;
        output VLEFluidPointer vleFluidPointer;
        external "C" vleFluidPointer = TILMedia_VLEFluid_createExternalObject(vleFluidName, flags, xi, nc_propertyCalculation, nc) annotation(Library="TILMedia341");
       end constructor;

       function destructor "free memory"
        input VLEFluidPointer vleFluidPointer;
        external "C" TILMedia_VLEFluid_destroyExternalObject(vleFluidPointer) annotation(Library="TILMedia341");
       end destructor;
    end VLEFluidPointer;

    function bubblePressure_Txi
      input SI.Temperature T "Temperature";
      input SI.MassFraction[:] xi "Mass fractions of the first nc-1 components";
      input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
      output SI.AbsolutePressure p_bubble "Pressure at bubble point";
    external "C" p_bubble = TILMedia_VLEFluidObjectFunctions_bubblePressure_Txi(T, xi, vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/VLE_Function.png")}));
    end bubblePressure_Txi;

    function molarMass_n
      input Integer compNo "Component ID";
      input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
      output SI.MolarMass M_i "Molar mass of component i";
    external "C" M_i = TILMedia_VLEFluidObjectFunctions_molarMass_n(compNo, vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/VLE_Function.png")}));
    end molarMass_n;
  end VLEFluidObjectFunctions;

  package VLEFluidTypes
  "VLEFluids and VLEFluid mixtures, that can be used or composed in TILMedia"
  extends TILMedia.Internals.ClassTypes.ModelPackage;

    record BaseVLEFluid "Base record for VLE Fluid definitions"
      extends Internals.ClassTypes.Record;
      constant Boolean fixedMixingRatio
        "Treat medium as pseudo pure in Modelica if it is a mixture" annotation(HideResult = true);
      constant Integer nc_propertyCalculation(min=1)
        "Number of components for fluid property calculations" annotation(HideResult = true);
      final constant Integer nc=if fixedMixingRatio then 1 else
          nc_propertyCalculation "Number of components in Modelica models"
        annotation (Evaluate=true, HideResult = true);
      constant Internals.VLEFluidName[nc_propertyCalculation] vleFluidNames
        "Array of VLEFluid names" annotation (choices);
      constant String concatVLEFluidName=TILMedia.Internals.concatNames(
          vleFluidNames);
      constant Real[nc_propertyCalculation] mixingRatio_propertyCalculation
        "Mixing ratio for fluid property calculation (={1} for pure components)" annotation(HideResult = true);
      constant Real[nc] defaultMixingRatio=if fixedMixingRatio then {1} else
          mixingRatio_propertyCalculation
        "Default composition for models in Modelica (={1} for pure components)" annotation(HideResult = true);
      constant Real xi_default[nc-1] = defaultMixingRatio[1:end-1]/sum(defaultMixingRatio)
        "Default mass fractions" annotation(HideResult = true);
      constant Integer ID=0
        "ID is used to map the selected VLEFluid to the sim.cumulatedVLEFluidMass array item" annotation(HideResult = true);
    end BaseVLEFluid;

    record TILMedia_CO2 "TILMedia.CO2"
      extends TILMedia.VLEFluidTypes.BaseVLEFluid(
        final fixedMixingRatio=true,
        final nc_propertyCalculation=1,
        final vleFluidNames={"TILMedia.CO2"},
        final mixingRatio_propertyCalculation={1});
    end TILMedia_CO2;
  end VLEFluidTypes;

  package Internals "Internal functions"

    package ClassTypes "Icon definitions"

      partial class ModelPackage

       annotation (Icon(graphics={
              Rectangle(
                extent={{-100,-100},{80,50}},
                lineColor={0,0,0},
                fillColor={255,204,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
                lineColor={0,0,0},
                fillColor={255,204,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
                lineColor={0,0,0},
                fillColor={255,204,0},
                fillPattern=FillPattern.Solid)}));
      end ModelPackage;

      partial record Record "Partial Record"

        annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,60}},
                imageSource=
                    "iVBORw0KGgoAAAANSUhEUgAAAGQAAABkCAYAAAEH5aXCAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA2dJREFUeNpilBKX/s9AADx7+ZQRmc8CIp6+eEJI339GIEDRBAIHXv/BqcNBlIXhPxDANDIxkABAGlFsAplGrEainIfuVJKcBwNkaSLZT0QH+RDzE0AAMZKcYP8TAZANZSEmxECpX1pC5j/IJqI9D9IEsokRllJJCimaRiLJGkakHwACCBx71AD4EjFKXiA2wHAlN1D6B6ZTjCKciYHKAGQZ1pxEDQDyAbaogBcIRFRGFAGqxQmuxE6TOCHoE1IKXppXB6NxQtM4AQggoipOIsAHYFEiSL0SFXdux2kR1eIEWGoIAEPlPVXaToQsAvrqPbqPaFEKY/iIJkkY3SKa5RNki2hanwAB2CKqJGFCltO3WKFV4Ui3AnLUEtp3rEaDazSfjOaTIeoTgADsmVEKgDAMQ/vhpdwxhJ1y4BUniJ86jWwlHckNHum6tD3DXdmLMShv+TUoNi1JaTUWXdGtokC3BdW7wfzcCEBAixELAaIGQYBCgHwBCgXSAnLZQY2W21wyTX5wzShM/wgSfKdzRCAC8XzsI4cTOaL2q9ISiEDUfmkdebjWhNIhALt2dIMgDEVhmH0g0WUMz0b3UccgLIORReoE0j7xgNI2JZxr/m7w0ZJ77m1L3b6WWsepbX1mQxQmjW/nqsv5mo2R6tlnRzwZI/ezn9ow/x2mD3wwDbk/blkYydDoMX71Xe8xUcdMto6k7ox0QUzByFf2WIyJiBKDMZO11jCmQuMvjLn0+w1jMsYvYeQ6qIyWIhTNYg/A91rjawxPtM13iHVTM0UBsvnwQS3Gry1mv0CAAAECBEhKZeeiBwgQYjwxnqMFBAgQIECI8ewIEOP/yD+8oPsI0M4d3SAIQ2EYJcYBHMEFnMgh1MllBNOILyYSlBvyg+cbgZO2cq2m3aBLq29736+3+maBrP2Hb9W9rbbFYHYe/XjDtYRD9/w2//7tBUUgxbW7Im33WAoGSBjM5DMkfV5X3fl07Maey/Vyazd5y88YKyRsxQAJgwESBgMkDAZIGAyQMBggYTBAwmD2HtF4C430XzC9aW8YvC0rLCBABGRFTf6U9W/j9+qmXkqyQmxZAgJEQIAICBAB8aZe+6YpKwSIgABR+KFu/D4v43dbloAAERAgAiIgQFT7pm78boUAERB9OkO28G9sW+gB4BfZ2hOOJl8AAAAASUVORK5CYII=",
                fileName="modelica://TILMedia/Images/recordIcon.png"),
                                            Text(
                extent={{-100,110},{100,70}},
                lineColor={0,0,0},
                textString=
                     "%name")}));

      end Record;
    annotation (Icon(graphics={
            Rectangle(
              extent={{-100,-100},{80,50}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end ClassTypes;

    package GasObjectFunctions
        extends TILMedia.Internals.ClassTypes.ModelPackage;

      function pureComponentProperties_Tnc
        input SI.Temperature T "Temperature";
        input Integer nc "Number of components";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.PartialPressure ppS
          "Saturation partial pressure of condensing component";
        output SI.SpecificEnthalpy delta_hv
          "Specific enthalpy of vaporation of condensing component";
        output SI.SpecificEnthalpy delta_hd
          "Specific enthalpy of desublimation of condensing component";
        output SI.SpecificEnthalpy h_idealGas[nc]
          "Specific enthalpy of theoretical pure component ideal gas state";
        external "C" TILMedia_Gas_pureComponentProperties_T(T,gasPointer,ppS,delta_hv,delta_hd,h_idealGas) annotation(Library="TILMedia341");
        annotation(Impure=false);
      end pureComponentProperties_Tnc;

      function simpleCondensingProperties_phxi
        input SI.AbsolutePressure p "Pressure";
        input SI.SpecificEnthalpy h "Specific enthalpy";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.SpecificHeatCapacity cp "Specific heat capacity cp";
        output SI.SpecificHeatCapacity cv "Specific heat capacity cv";
        output SI.LinearExpansionCoefficient beta
          "Isothermal expansion coefficient";
        output SI.Velocity w "Speed of sound";
        external "C" TILMedia_Gas_simpleCondensingProperties_phxi(p,h,xi,gasPointer,cp,cv,beta,w) annotation(Library="TILMedia341");
        annotation(Impure=false);
      end simpleCondensingProperties_phxi;

      function simpleCondensingProperties_pTxi
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.SpecificHeatCapacity cp "Specific heat capacity cp";
        output SI.SpecificHeatCapacity cv "Specific heat capacity cv";
        output SI.LinearExpansionCoefficient beta
          "Isothermal expansion coefficient";
        output SI.Velocity w "Speed of sound";
        external "C" TILMedia_Gas_simpleCondensingProperties_pTxi(p,T,xi,gasPointer,cp,cv,beta,w) annotation(Library="TILMedia341");
        annotation(Impure=false);
      end simpleCondensingProperties_pTxi;

      function additionalProperties_pTxi
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.Density d "Density";
        output SI.Compressibility kappa "Compressibility";
        output SI.DerDensityByPressure drhodp_hxi
          "Derivative of density wrt pressure";
        output SI.DerDensityByEnthalpy drhodh_pxi
          "Derivative of density wrt specific enthalpy";
        output SI.Density drhodxi_ph[size(xi,1)]
          "Derivative of density wrt mass fraction";
        output SI.PartialPressure pp[size(xi,1)+1] "Partial pressure";
        output SI.MassFraction xi_gas
          "Mass fraction of gasoues condensing component";
        external "C" TILMedia_Gas_additionalProperties_pTxi(p,T,xi,gasPointer,d,kappa,drhodp_hxi,drhodh_pxi,drhodxi_ph,pp,xi_gas) annotation(Library="TILMedia341");
        annotation(Impure=false);
      end additionalProperties_pTxi;

      function transportProperties_pTxi
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output TILMedia.Internals.TransportPropertyRecord transp
          "Transport property record";
        external "C" TILMedia_Gas_transportProperties_pTxi(p,T,xi,gasPointer,transp.Pr,transp.lambda,transp.eta,transp.sigma) annotation(Library="TILMedia341");
        annotation(Impure=false);
      end transportProperties_pTxi;

      function temperature_phxi
        input SI.AbsolutePressure p "Pressure";
        input SI.SpecificEnthalpy h "Specific enthalpy";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.Temperature T "Temperature";
        external "C" T=TILMedia_Gas_temperature_phxi(p,h,xi,gasPointer)
        annotation(Library="TILMedia341", inverse(h=specificEnthalpy_pTxi(p,T,xi,gasPointer)));
      end temperature_phxi;

      function temperature_psxi
        input SI.AbsolutePressure p "Pressure";
        input SI.SpecificEntropy s "Specific entropy";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.Temperature T "Temperature";
        external "C" T=TILMedia_Gas_temperature_psxi(p,s,xi,gasPointer)
        annotation(Library="TILMedia341", inverse(s=specificEntropy_pTxi(p,T,xi,gasPointer)));
      end temperature_psxi;

      function specificEnthalpy_psxi
        input SI.AbsolutePressure p "Pressure";
        input SI.SpecificEntropy s "Specific entropy";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.SpecificEnthalpy h "Specific enthalpy";
        external "C" h=TILMedia_Gas_specificEnthalpy_psxi(p,s,xi,gasPointer)
        annotation(Library="TILMedia341", inverse(s=specificEntropy_phxi(p,h,xi,gasPointer)));
      end specificEnthalpy_psxi;

      function specificEnthalpy_pTxi
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.SpecificEnthalpy h "Specific enthalpy";
        external "C" h=TILMedia_Gas_specificEnthalpy_pTxi(p,T,xi,gasPointer)
        annotation(Library="TILMedia341", inverse(T=temperature_phxi(p,h,xi,gasPointer)));
      end specificEnthalpy_pTxi;

      function specificEntropy_pTxi
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.SpecificEntropy s "Specific entropy";
        external "C" s=TILMedia_Gas_specificEntropy_pTxi(p,T,xi,gasPointer)
        annotation(Library="TILMedia341", inverse(T=temperature_psxi(p,s,xi,gasPointer)));
      end specificEntropy_pTxi;

      function specificEntropy_phxi
        input SI.AbsolutePressure p "Pressure";
        input SI.SpecificEnthalpy h "Specific enthalpy";
        input SI.MassFraction xi[:] "Mass fraction";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.SpecificEntropy s "Specific entropy";
        external "C" s=TILMedia_Gas_specificEntropy_phxi(p,h,xi,gasPointer)
        annotation(Library="TILMedia341", inverse(h=specificEnthalpy_psxi(p,s,xi,gasPointer)));
      end specificEntropy_phxi;

      function xi_s_pTxidg
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi_dryGas[:] "Mass fraction of dry gas";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output SI.MassFraction xi_s "Saturation vapour mass fraction";
        external "C" xi_s=TILMedia_Gas_saturationMassFraction_pTxidg(p,T,xi_dryGas,gasPointer)
        annotation(Library="TILMedia341");
      end xi_s_pTxidg;

      function humRatio_s_pTxidg
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction xi_dryGas[:] "Mass fraction of dry gas";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output Real humRatio_s "Saturation humidity ratio";
        external "C" humRatio_s=TILMedia_Gas_saturationHumidityRatio_pTxidg(p,T,xi_dryGas,gasPointer)
        annotation(Library="TILMedia341");
      end humRatio_s_pTxidg;

      function phi_pThumRatioxidg
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input Real humRatio "Humidity ratio";
        input SI.MassFraction xi_dryGas[:] "Mass fraction of dry gas";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output TILMedia.Internals.Units.RelativeHumidity phi "Relative humidity";
        external "C" phi=TILMedia_MoistAir_phi_pThumRatioxidg(p,T,humRatio,xi_dryGas,gasPointer)
        annotation(Library="TILMedia341", inverse(humRatio=humRatio_pTphixidg(p,T,phi,xi_dryGas,gasPointer)));
      end phi_pThumRatioxidg;

      function humRatio_pTphixidg
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input TILMedia.Internals.Units.RelativeHumidity phi "Relative humidity";
        input SI.MassFraction xi_dryGas[:] "Mass fraction of dry gas";
        input TILMedia.GasObjectFunctions.GasPointer gasPointer;
        output Real humRatio "Humidity ratio";
        external "C" humRatio=TILMedia_MoistAir_humRatio_pTphixidg(p,T,phi,xi_dryGas,gasPointer)
        annotation(Library="TILMedia341", inverse(phi=phi_pThumRatioxidg(p,T,humRatio,xi_dryGas,gasPointer)));
      end humRatio_pTphixidg;
    end GasObjectFunctions;

    package GasFunctions
      extends TILMedia.Internals.ClassTypes.ModelPackage;

    function density_pTxi
      input SI.AbsolutePressure p "Pressure";
      input SI.Temperature T "Temperature";
      input SI.MassFraction[:] xi "Mass fractions of the first nc-1 components";
      input TILMedia.Internals.GasName gasName "Gas name";
      input Integer nc "Number of components";
      input Integer condensingIndex "Index of condensing component";
      output SI.Density d "Density";
    external "C" d = TILMedia_GasFunctions_density_pTxi(p, T, xi, gasName, nc, condensingIndex)
      annotation(Library="TILMedia341");
      annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/Gas_Function.png")}));
    end density_pTxi;

    function specificEnthalpy_pTxi
      input SI.AbsolutePressure p "Pressure";
      input SI.Temperature T "Temperature";
      input SI.MassFraction[:] xi "Mass fractions of the first nc-1 components";
      input TILMedia.Internals.GasName gasName "Gas name";
      input Integer nc "Number of components";
      input Integer condensingIndex "Index of condensing component";
      output SI.SpecificEnthalpy h "Specific enthalpy";
    external "C" h = TILMedia_GasFunctions_specificEnthalpy_pTxi(p, T, xi, gasName, nc, condensingIndex)
      annotation(Library="TILMedia341");
      annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/Gas_Function.png")}));
    end specificEnthalpy_pTxi;
    end GasFunctions;

    package VLEFluidObjectFunctions
      extends TILMedia.Internals.ClassTypes.ModelPackage;

      function additionalProperties_phxi
        input Modelica.SIunits.AbsolutePressure p "Pressure";
        input Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
        input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
        input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer
                           vleFluidPointer;
        output Modelica.SIunits.MassFraction x "Steam mass fraction";
        output Modelica.SIunits.SpecificHeatCapacity cp "Specific heat capacity cp";
        output Modelica.SIunits.SpecificHeatCapacity cv "Specific heat capacity cv";
        output Modelica.SIunits.LinearExpansionCoefficient beta
          "Isobaric expansion coefficient";
        output Modelica.SIunits.Compressibility kappa "Isothermal compressibility";
        output TILMedia.Internals.Units.DensityDerPressure drhodp
          "Derivative of density wrt pressure";
        output TILMedia.Internals.Units.DensityDerSpecificEnthalpy drhodh
          "Derivative of density wrt specific enthalpy";
        output Real[size(xi,1)] drhodxi "Derivative of density wrt mass fraction";
        output Modelica.SIunits.Velocity a "Speed of sound";
        output Real gamma "Heat capacity ratio";
      external "C" TILMedia_VLEFluid_additionalProperties_phxi(p,h,xi,vleFluidPointer,x,cp,cv,beta,kappa,drhodp,drhodh,drhodxi,a,gamma)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end additionalProperties_phxi;

      function transportPropertyRecord_phxi
        input Modelica.SIunits.AbsolutePressure p "Pressure";
        input Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
        input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
        input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer
                           vleFluidPointer;
        output TILMedia.Internals.TransportPropertyRecord transp
          "Transport property record";
      external "C" TILMedia_VLEFluid_transportProperties_phxi(p,h,xi,vleFluidPointer,transp.Pr,transp.lambda,transp.eta,transp.sigma)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end transportPropertyRecord_phxi;

      function VLETransportPropertyRecord_phxi
        input Modelica.SIunits.AbsolutePressure p "Pressure";
        input Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
        input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
        input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer
                           vleFluidPointer;
        output Modelica.SIunits.PrandtlNumber Pr_l "Prandtl number";
        output Modelica.SIunits.PrandtlNumber Pr_v "Prandtl number";
        output Modelica.SIunits.ThermalConductivity lambda_l "Thermal conductivity";
        output Modelica.SIunits.ThermalConductivity lambda_v "Thermal conductivity";
        output Modelica.SIunits.DynamicViscosity eta_l "Dynamic viscosity";
        output Modelica.SIunits.DynamicViscosity eta_v "Dynamic viscosity";
      external "C" TILMedia_VLEFluid_VLETransportProperties_phxi(p, h, xi,vleFluidPointer, Pr_l, Pr_v, lambda_l, lambda_v, eta_l, eta_v)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end VLETransportPropertyRecord_phxi;

      function VLEAdditionalProperties_phxi
        input Modelica.SIunits.AbsolutePressure p "Pressure";
        input Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
        input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
        input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer
                           vleFluidPointer;
        output Modelica.SIunits.SpecificHeatCapacity cp_l
          "Specific heat capacity cp";
        output Modelica.SIunits.LinearExpansionCoefficient beta_l
          "Isobaric expansion coefficient";
        output Modelica.SIunits.Compressibility kappa_l
          "Isothermal compressibility";
        output Modelica.SIunits.SpecificHeatCapacity cp_v
          "Specific heat capacity cp";
        output Modelica.SIunits.LinearExpansionCoefficient beta_v
          "Isobaric expansion coefficient";
        output Modelica.SIunits.Compressibility kappa_v
          "Isothermal compressibility";
      external "C" TILMedia_VLEFluid_VLEAdditionalProperties_phxi(p,h,xi,vleFluidPointer,
        cp_l, beta_l, kappa_l,
        cp_v, beta_v, kappa_v)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end VLEAdditionalProperties_phxi;

      function VLEProperties_phxi
        input Modelica.SIunits.AbsolutePressure p "Pressure";
        input Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy";
        input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
        input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer
                           vleFluidPointer;
        output Modelica.SIunits.Density d_l "Density";
        output Modelica.SIunits.SpecificEnthalpy h_l "Specific enthalpy";
        output Modelica.SIunits.AbsolutePressure p_l "Pressure";
        output Modelica.SIunits.SpecificEntropy s_l "Specific entropy";
        output Modelica.SIunits.Temperature T_l "Temperature";
        output Modelica.SIunits.MassFraction[size(xi,1)] xi_l "Mass fractions";
        output Modelica.SIunits.Density d_v "Density";
        output Modelica.SIunits.SpecificEnthalpy h_v "Specific enthalpy";
        output Modelica.SIunits.AbsolutePressure p_v "Pressure";
        output Modelica.SIunits.SpecificEntropy s_v "Specific entropy";
        output Modelica.SIunits.Temperature T_v "Temperature";
        output Modelica.SIunits.MassFraction[size(xi,1)] xi_v "Mass fractions";
      external "C" TILMedia_VLEFluid_VLEProperties_phxi(p,h,xi,vleFluidPointer,
        d_l, h_l, p_l, s_l, T_l, xi_l,
        d_v, h_v, p_v, s_v, T_v, xi_v)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end VLEProperties_phxi;

      function specificEnthalpy_pTxi
       input SI.AbsolutePressure p "Pressure";
       input SI.Temperature T "Temperature";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.SpecificEnthalpy h "Specific Enthalpy";
      external "C" h = TILMedia_VLEFluid_Cached_specificEnthalpy_pTxi(p,T,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_specificEnthalpy_pTxi, inverse(T=temperature_phxi(p, h, xi, vleFluidPointer)),Impure=false);
      end specificEnthalpy_pTxi;

      function density_pTxi
       input SI.AbsolutePressure p "Pressure";
       input SI.Temperature T "Temperature";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.Density d "Density";
      external "C" d = TILMedia_VLEFluid_Cached_density_pTxi(p,T,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_density_pTxi);
      end density_pTxi;

      function specificEntropy_pTxi
       input SI.AbsolutePressure p "Pressure";
       input SI.Temperature T "Temperature";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.SpecificEntropy s "Specific Entropy";
      external "C" s = TILMedia_VLEFluid_Cached_specificEntropy_pTxi(p,T,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_specificEntropy_pTxi, inverse(T=temperature_psxi(p, s, xi, vleFluidPointer)),Impure=false);
      end specificEntropy_pTxi;

      function density_phxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEnthalpy h "Specific Enthalpy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.Density d "Density";
      external "C" d = TILMedia_VLEFluid_Cached_density_phxi(p,h,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_density_phxi,Impure=false);
      end density_phxi;

      function specificEntropy_phxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEnthalpy h "Specific Enthalpy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.SpecificEntropy s "Specific Entropy";
      external "C" s = TILMedia_VLEFluid_Cached_specificEntropy_phxi(p,h,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_specificEntropy_phxi, inverse(h=specificEnthalpy_psxi(p, s, xi, vleFluidPointer)),Impure=false);
      end specificEntropy_phxi;

      function temperature_phxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEnthalpy h "Specific Enthalpy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.Temperature T "Temperature";
      external "C" T = TILMedia_VLEFluid_Cached_temperature_phxi(p,h,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_temperature_phxi, inverse(h=specificEnthalpy_pTxi(p, T, xi, vleFluidPointer)),Impure=false);
      end temperature_phxi;

      function specificEnthalpy_psxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEntropy s "Specific Entropy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.SpecificEnthalpy h "Specific Enthalpy";
      external "C" h = TILMedia_VLEFluid_Cached_specificEnthalpy_psxi(p,s,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_specificEnthalpy_psxi, inverse(s=specificEntropy_phxi(p, h, xi, vleFluidPointer)),Impure=false);
      end specificEnthalpy_psxi;

      function density_psxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEntropy s "Specific Entropy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.Density d "Density";
      external "C" d = TILMedia_VLEFluid_Cached_density_psxi(p,s,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_density_psxi,Impure=false);
      end density_psxi;

      function temperature_psxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEntropy s "Specific Entropy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       output SI.Temperature T "Temperature";
      external "C" T = TILMedia_VLEFluid_Cached_temperature_psxi(p,s,xi,vleFluidPointer)
      annotation(Library="TILMedia341");
      annotation(derivative=der_temperature_psxi, inverse(s=specificEntropy_pTxi(p, T, xi, vleFluidPointer)),Impure=false);
      end temperature_psxi;

      function der_specificEnthalpy_pTxi
       input SI.AbsolutePressure p "Pressure";
       input SI.Temperature T "Temperature";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_T "Derivative of Temperature";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_h "Derivative of Specific Enthalpy";
      external "C" der_h = TILMedia_VLEFluid_Cached_der_specificEnthalpy_pTxi(p,T,xi,der_p,der_T,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_specificEnthalpy_pTxi;

      function der_density_pTxi
       input SI.AbsolutePressure p "Pressure";
       input SI.Temperature T "Temperature";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_T "Derivative of Temperature";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_d "Derivative of Density";
      external "C" der_d = TILMedia_VLEFluid_Cached_der_density_pTxi(p,T,xi,der_p,der_T,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_density_pTxi;

      function der_specificEntropy_pTxi
       input SI.AbsolutePressure p "Pressure";
       input SI.Temperature T "Temperature";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_T "Derivative of Temperature";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_s "Derivative of Specific Entropy";
      external "C" der_s = TILMedia_VLEFluid_Cached_der_specificEntropy_pTxi(p,T,xi,der_p,der_T,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_specificEntropy_pTxi;

      function der_density_phxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEnthalpy h "Specific Enthalpy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_h "Derivative of Specific Enthalpy";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_d "Derivative of Density";
      external "C" der_d = TILMedia_VLEFluid_Cached_der_density_phxi(p,h,xi,der_p,der_h,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_density_phxi;

      function der_specificEntropy_phxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEnthalpy h "Specific Enthalpy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_h "Derivative of Specific Enthalpy";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_s "Derivative of Specific Entropy";
      external "C" der_s = TILMedia_VLEFluid_Cached_der_specificEntropy_phxi(p,h,xi,der_p,der_h,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_specificEntropy_phxi;

      function der_temperature_phxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEnthalpy h "Specific Enthalpy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_h "Derivative of Specific Enthalpy";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_T "Derivative of Temperature";
      external "C" der_T = TILMedia_VLEFluid_Cached_der_temperature_phxi(p,h,xi,der_p,der_h,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_temperature_phxi;

      function der_specificEnthalpy_psxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEntropy s "Specific Entropy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_s "Derivative of Specific Entropy";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_h "Derivative of Specific Enthalpy";
      external "C" der_h = TILMedia_VLEFluid_Cached_der_specificEnthalpy_psxi(p,s,xi,der_p,der_s,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_specificEnthalpy_psxi;

      function der_density_psxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEntropy s "Specific Entropy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_s "Derivative of Specific Entropy";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_d "Derivative of Density";
      external "C" der_d = TILMedia_VLEFluid_Cached_der_density_psxi(p,s,xi,der_p,der_s,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_density_psxi;

      function der_temperature_psxi
       input SI.AbsolutePressure p "Pressure";
       input SI.SpecificEntropy s "Specific Entropy";
       input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
       input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer;
       input Real der_p "Derivative of Pressure";
       input Real der_s "Derivative of Specific Entropy";
       input Real[:] der_xi
          "Derivative of Mass fractions of the first nc-1 components";
       output Real der_T "Derivative of Temperature";
      external "C" der_T = TILMedia_VLEFluid_Cached_der_temperature_psxi(p,s,xi,der_p,der_s,der_xi,vleFluidPointer)
      annotation(Library="TILMedia341");
                                        annotation(Impure=false);
      end der_temperature_psxi;

      function cricondenbar_xi
        input Modelica.SIunits.MassFraction[:] xi
          "Mass fractions of the first nc-1 components";
        input TILMedia.VLEFluidObjectFunctions.VLEFluidPointer
                           vleFluidPointer;
        output Real d,h,p,s,T;
      external "C" TILMedia_VLEFluid_cricondenbar_xi(xi, vleFluidPointer,d,h,p,s,T) annotation(Library="TILMedia341");
                                                                                                            annotation(Impure=false);
      end cricondenbar_xi;
    end VLEFluidObjectFunctions;

    package VLEFluidFunctions
      extends TILMedia.Internals.ClassTypes.ModelPackage;

      function specificEnthalpy_pTxi
        input SI.AbsolutePressure p "Pressure";
        input SI.Temperature T "Temperature";
        input SI.MassFraction[:] xi "Mass fractions of the first nc-1 components";
        input TILMedia.Internals.VLEFluidName vleFluidName "VLEFluid name";
        input Integer nc "Number of components";
        output SI.SpecificEnthalpy h "Specific enthalpy";
      external "C" h = TILMedia_VLEFluidFunctions_specificEnthalpy_pTxi(p, T, xi, vleFluidName, nc)
        annotation(Library="TILMedia341");
        annotation (Icon(graphics={Bitmap(extent={{-100,-100},{100,100}}, fileName="modelica://TILMedia/Images/VLE_Function.png")}));
      end specificEnthalpy_pTxi;
    end VLEFluidFunctions;

    package Units "Unit definitions"
    extends TILMedia.Internals.ClassTypes.ModelPackage;

      type DensityDerPressure = Real(final unit="kg/(N.m)");

      type DensityDerSpecificEnthalpy = Real(final unit="kg2/(m3.J)");

      type DensityDerMassFraction =     Real(final unit="kg/(m3)");

      type RelativeHumidity = Real(final unit="1", min=0, max=100);
    end Units;

    type GasName "Gas name"
      extends String;
      annotation(choices(
         choice="TILMedia.DryAir",
         choice="TILMedia.ExhaustGas_Lambda_1",
         choice="TILMedia.DIESELEXHAUSTGAS",
         choice="TILMedia.ASH",
       choice="TILMediaXTR.AMMONIA",
       choice="TILMediaXTR.ARGON",
       choice="TILMediaXTR.CARBON_MONOXIDE",
       choice="TILMediaXTR.CARBON_DIOXIDE",
       choice="TILMediaXTR.DRYAIR",
       choice="TILMediaXTR.HYDROGEN",
       choice="TILMediaXTR.NITROGEN",
       choice="TILMediaXTR.NITROUS_OXIDE",
       choice="TILMediaXTR.OXYGEN",
       choice="TILMediaXTR.SULFUR_DIOXIDE",
       choice="TILMediaXTR.WATER",
      choice="VDI4670.ARGON",
      choice="VDI4670.CARBON_DIOXIDE",
      choice="VDI4670.CARBON_MONOXIDE",
      choice="VDI4670.DRYAIR",
      choice="VDI4670.NEON",
      choice="VDI4670.NITROGEN",
      choice="VDI4670.OXYGEN",
      choice="VDI4670.SULPHUR_DIOXIDE",
      choice="VDI4670.WATER",
     choice="VDIWA2006.Xenon",
     choice="VDIWA2006.Krypton",
     choice="VDIWA2006.Argon",
     choice="VDIWA2006.Neon",
     choice="VDIWA2006.Helium",
     choice="VDIWA2006.DryAir",
     choice="VDIWA2006.Hydrogen",
     choice="VDIWA2006.Nitrogen",
     choice="VDIWA2006.Oxygen",
     choice="VDIWA2006.Sulfur",
     choice="VDIWA2006.Fluorine",
     choice="VDIWA2006.Chlorine",
     choice="VDIWA2006.Bromine",
     choice="VDIWA2006.Iodine",
     choice="VDIWA2006.Hydrogen fluoride",
     choice="VDIWA2006.Hydrogen chloride",
     choice="VDIWA2006.Hydrogen bromide",
     choice="VDIWA2006.Hydrogen Iodide",
     choice="VDIWA2006.Hydrogen cyanide",
     choice="VDIWA2006.Water",
     choice="VDIWA2006.Hydrogen sulfide",
     choice="VDIWA2006.Ammonia",
     choice="VDIWA2006.Nitric oxide",
     choice="VDIWA2006.Nitrogen dioxide",
     choice="VDIWA2006.Nitrous oxide",
     choice="VDIWA2006.Dinitrogen tetroxide",
     choice="VDIWA2006.Cyanogen",
     choice="VDIWA2006.Cyanogen fluoride",
     choice="VDIWA2006.Cyanogen chloride",
     choice="VDIWA2006.Cyanogen bromide",
     choice="VDIWA2006.Iodine cyanide",
     choice="VDIWA2006.carbon monoxide",
     choice="VDIWA2006.carbon dioxide",
     choice="VDIWA2006.carbon suboxide",
     choice="VDIWA2006.carbonyl sulfide",
     choice="VDIWA2006.phosgene",
     choice="VDIWA2006.carbon disulfide",
     choice="VDIWA2006.sulfur dioxide",
     choice="VDIWA2006.sulfur trioxide",
     choice="VDIWA2006.sulfury chloride",
     choice="VDIWA2006.sulfur hexafluoride",
     choice="VDIWA2006.methanethiol",
     choice="VDIWA2006.ethanethiol",
     choice="VDIWA2006.dimethyl sulfide",
     choice="VDIWA2006.diethyl sulfide",
     choice="VDIWA2006.thiophene",
     choice="VDIWA2006.fluoromethane",
     choice="VDIWA2006.difluoromethane",
     choice="VDIWA2006.trifluoromethane/fluoroform",
     choice="VDIWA2006.tetrafluoromethane",
     choice="VDIWA2006.chloromethane",
     choice="VDIWA2006.dichloromethane",
     choice="VDIWA2006.methane trichloride/chloroform",
     choice="VDIWA2006.carbon tetrachloride",
     choice="VDIWA2006.bromomethane",
     choice="VDIWA2006.dibromomethane",
     choice="VDIWA2006.tribromomethane/bromoform",
     choice="VDIWA2006.tetrabromomethane",
     choice="VDIWA2006.chlorodifluoromethane",
     choice="VDIWA2006.dichlorodifluoromethane",
     choice="VDIWA2006.chlorotrifluoromethane",
     choice="VDIWA2006.dichlorodifluoromethane",
     choice="VDIWA2006.trichlorofluoromethane",
     choice="VDIWA2006.fluoroethane",
     choice="VDIWA2006.chloroethane",
     choice="VDIWA2006.bromoethane",
     choice="VDIWA2006.methane",
     choice="VDIWA2006.ethane",
     choice="VDIWA2006.propane",
     choice="VDIWA2006.butane",
     choice="VDIWA2006.pentane",
     choice="VDIWA2006.hexane",
     choice="VDIWA2006.heptane",
     choice="VDIWA2006.octane",
     choice="VDIWA2006.nonane",
     choice="VDIWA2006.decane",
     choice="VDIWA2006.undecane",
     choice="VDIWA2006.dodecane",
     choice="VDIWA2006.tridecane ",
     choice="VDIWA2006.tetradecane ",
     choice="VDIWA2006.pentadecane ",
     choice="VDIWA2006.hexadecane",
     choice="VDIWA2006.heptadecane",
     choice="VDIWA2006.octadecane",
     choice="VDIWA2006.nonadecane",
     choice="VDIWA2006.icosane"));
    end GasName;

    type LiquidName "Liquid name"
      extends String;

      annotation(choices(
        choice="TILMedia.Water",
        choice="TILMedia.Glysantin_30",
        choice="TILMedia.Glysantin_40",
        choice="TILMedia.Glysantin_50",
        choice="TILMedia.Glysantin_60",
        choice="TILMedia.ADDINOLXW15",
        choice="TILMedia.Oil_15W40",
        choice="TILMedia.Oil_Aral0W30",
        choice="TILMedia.SHC_XMP320",
        choice="TILMedia.Propylenglykol_30",
        choice="TILMedia.Propylenglykol_40",
        choice="TILMedia.Propylenglykol_50",
        choice="TILMedia.TherminolD12",
        choice="TILMedia.Therminol59",
        choice="TILMedia.Therminol66",
        choice="TILMedia.Therminol72",
        choice="TILMedia.TYFOCOR30",
        choice="TILMedia.TYFOCOR45",
        choice="TILMedia.TYFOCORL33",
        choice="TILMedia.ZitrecM10",
        choice="TILMedia.ZitrecM20",
      choice="TILMediaXTR.AMMONIA",
      choice="TILMediaXTR.ARGON",
      choice="TILMediaXTR.CARBON_MONOXIDE",
      choice="TILMediaXTR.CARBON_DIOXIDE",
      choice="TILMediaXTR.DRYAIR",
      choice="TILMediaXTR.HYDROGEN",
      choice="TILMediaXTR.NITROGEN",
      choice="TILMediaXTR.NITROUS_OXIDE",
      choice="TILMediaXTR.OXYGEN",
      choice="TILMediaXTR.SULFUR_DIOXIDE",
      choice="TILMediaXTR.WATER",
     choice="VDIWA2006.XENON",
     choice="VDIWA2006.KRYPTON",
     choice="VDIWA2006.ARGON",
     choice="VDIWA2006.NEON",
     choice="VDIWA2006.HELIUM",
     choice="VDIWA2006.DRYAIR",
     choice="VDIWA2006.HYDROGEN",
     choice="VDIWA2006.NITROGEN",
     choice="VDIWA2006.OXYGEN",
     choice="VDIWA2006.SULFUR",
     choice="VDIWA2006.FLUORINE",
     choice="VDIWA2006.CHLORINE",
     choice="VDIWA2006.BROMINE",
     choice="VDIWA2006.IODINE",
     choice="VDIWA2006.HYDROGEN FLUORIDE",
     choice="VDIWA2006.HYDROGEN CHLORIDE",
     choice="VDIWA2006.HYDROGEN BROMIDE",
     choice="VDIWA2006.HYDROGEN IODIDE",
     choice="VDIWA2006.HYDROGEN CYANIDE",
     choice="VDIWA2006.WATER",
     choice="VDIWA2006.HYDROGEN SULFIDE",
     choice="VDIWA2006.AMMONIA",
     choice="VDIWA2006.NITRIC OXIDE",
     choice="VDIWA2006.NITROGEN DIOXIDE",
     choice="VDIWA2006.NITROUS OXIDE",
     choice="VDIWA2006.DINITROGEN TETROXIDE",
     choice="VDIWA2006.CYANOGEN",
     choice="VDIWA2006.FLUOROCYANIDE",
     choice="VDIWA2006.CHLOROCYANIDE",
     choice="VDIWA2006.BROMOCYANIDE",
     choice="VDIWA2006.IODINECYANIDE",
     choice="VDIWA2006.CARBON MONOXIDE",
     choice="VDIWA2006.CARBON DIOXIDE",
     choice="VDIWA2006.CARBON SUBOXIDE",
     choice="VDIWA2006.CARBONYL SULFIDE",
     choice="VDIWA2006.PHOSGENE",
     choice="VDIWA2006.CARBON DISULFIDE",
     choice="VDIWA2006.SULFUR DIOXIDE",
     choice="VDIWA2006.SULFUR TRIOXIDE",
     choice="VDIWA2006.SULFURY CHLORIDE",
     choice="VDIWA2006.SULFUR HEXAFLUORIDE",
     choice="VDIWA2006.METHANETHIOL",
     choice="VDIWA2006.ETHANETHIOL",
     choice="VDIWA2006.DIMETHYL SULFIDE",
     choice="VDIWA2006.DIETHYL SULFIDE",
     choice="VDIWA2006.THIOPHENE",
     choice="VDIWA2006.FLUOROMETHANE",
     choice="VDIWA2006.DIFLUOROMETHANE",
     choice="VDIWA2006.TRIFLUOROMETHANE",
     choice="VDIWA2006.TETRAFLUOROMETHANE",
     choice="VDIWA2006.CHLOROMETHANE",
     choice="VDIWA2006.DICHLOROMETHANE",
     choice="VDIWA2006.TRICHLOROMETHANE",
     choice="VDIWA2006.TETRACHLOROCARBON",
     choice="VDIWA2006.BROMOMETHANE",
     choice="VDIWA2006.DIBROMOMETHANE",
     choice="VDIWA2006.TRIBROMOMETHANE",
     choice="VDIWA2006.TETRABROMOMETHANE",
     choice="VDIWA2006.CHLORODIFLUOROMETHANE",
     choice="VDIWA2006.DICHLORODIFLUOROMETHANE",
     choice="VDIWA2006.CHLOROTRIFLUOROMETHANE",
     choice="VDIWA2006.DICHLORODIFLUOROMETHANE",
     choice="VDIWA2006.TRICHLOROFLUOROMETHANE",
     choice="VDIWA2006.FLUOROETHANE",
     choice="VDIWA2006.CHLOROETHANE",
     choice="VDIWA2006.BROMOETHANE",
     choice="VDIWA2006.1,1-DICHLOROETHANE",
     choice="VDIWA2006.1,2-DICHLOROETHANE",
     choice="VDIWA2006.1,2-DIBROMOETHANE",
     choice="VDIWA2006.1,1,1-TRIFLUOROETHANE",
     choice="VDIWA2006.1,1,1-TRICHLOROETHANE",
     choice="VDIWA2006.1,1,2,2-TETRACHLOROETHANE",
     choice="VDIWA2006.PENTACHLOROETHANE",
     choice="VDIWA2006.HEXACHLOROETHANE",
     choice="VDIWA2006.1,1,2,2-TETRACHLORODIFLUOROETHANE",
     choice="VDIWA2006.1,1,2-TRICHLORO-1,2,2-TRIFLUOROETHANE",
     choice="VDIWA2006.1,2-DICHLORO-1,1,2,2-TETRAFLUOROETHANE",
     choice="VDIWA2006.1-CHLOROPROPANE",
     choice="VDIWA2006.1-CHLOROBUTANE",
     choice="VDIWA2006.1-CHLOROPENTANE",
     choice="VDIWA2006.CHLOROTRIFLUOROETHYLENE",
     choice="VDIWA2006.CHLOROETHYLENE",
     choice="VDIWA2006.1,1-DICHLOROETHENE",
     choice="VDIWA2006.TRICHLOROETHYLENE",
     choice="VDIWA2006.TETRACHLOROETHENE",
     choice="VDIWA2006.FLUOROBENZENE",
     choice="VDIWA2006.CHLOROBENZENE",
     choice="VDIWA2006.BROMOBENZENE",
     choice="VDIWA2006.IODOBENZENE",
     choice="VDIWA2006.M-CHLOROTOLUENE",
     choice="VDIWA2006.CHLOROMETHYLBENZENE",
     choice="VDIWA2006.METHANE",
     choice="VDIWA2006.ETHANE",
     choice="VDIWA2006.PROPANE",
     choice="VDIWA2006.BUTANE",
     choice="VDIWA2006.PENTANE",
     choice="VDIWA2006.HEXANE",
     choice="VDIWA2006.HEPTANE",
     choice="VDIWA2006.OCTANE",
     choice="VDIWA2006.NONANE",
     choice="VDIWA2006.DECANE",
     choice="VDIWA2006.UNDECANE",
     choice="VDIWA2006.DODECANE",
     choice="VDIWA2006.TRIDECANE",
     choice="VDIWA2006.TETRADECANE",
     choice="VDIWA2006.PENTADECANE",
     choice="VDIWA2006.HEXADECANE",
     choice="VDIWA2006.HEPTADECANE",
     choice="VDIWA2006.OCTADECANE",
     choice="VDIWA2006.NONADECANE",
     choice="VDIWA2006.ICOSANE",
     choice="VDIWA2006.2-METHYLPROPANE",
     choice="VDIWA2006.2-METHYLBUTANE",
     choice="VDIWA2006.2,2-DIMETHYLPROPANE",
     choice="VDIWA2006.2-METHYLPENTANE",
     choice="VDIWA2006.3-METHYLPENTANE",
     choice="VDIWA2006.2,2-DIMETHYLBUTANE",
     choice="VDIWA2006.2,3-DIMETHYLBUTANE",
     choice="VDIWA2006.ETHENE",
     choice="VDIWA2006.PROPENE",
     choice="VDIWA2006.1-BUTENE",
     choice="VDIWA2006.1-PENTENE",
     choice="VDIWA2006.1-HEXENE",
     choice="VDIWA2006.1-HEPTENE",
     choice="VDIWA2006.1-OCTENE",
     choice="VDIWA2006.DIMETHYLENEMETHANE",
     choice="VDIWA2006.1,2-BUTADIENE",
     choice="VDIWA2006.1,3-BUTADIENE",
     choice="VDIWA2006.1,2-PENTADIENE",
     choice="VDIWA2006.1,3-PENTADIENE",
     choice="VDIWA2006.1,4-PENTADIENE",
     choice="VDIWA2006.2,3-PENTADIENE",
     choice="VDIWA2006.ACETYLENE",
     choice="VDIWA2006.METHYLACETYLENE",
     choice="VDIWA2006.DIMETHYLACETYLENE",
     choice="VDIWA2006.ETHYLACETYLENE",
     choice="VDIWA2006.CYCLOPROPANE",
     choice="VDIWA2006.CYCLOBUTANE",
     choice="VDIWA2006.CYCLOPENTANE",
     choice="VDIWA2006.METHYLCYCLOPENTANE",
     choice="VDIWA2006.ETHYLCYCLEPENTANE",
     choice="VDIWA2006.PROPYLCYCLOPENTANE",
     choice="VDIWA2006.BUTYLCYCLOPENTANE",
     choice="VDIWA2006.PENTYLCYCLOPENTANE",
     choice="VDIWA2006.HEXYLCYCLOPENTANE",
     choice="VDIWA2006.CYCLOHEXANE",
     choice="VDIWA2006.METHYLCYCLOHEXANE",
     choice="VDIWA2006.ETHYLCYCLOHEXANE",
     choice="VDIWA2006.PROPYLCYCLOHEXANE",
     choice="VDIWA2006.BUTYLCYCLOHEXANE",
     choice="VDIWA2006.PENTYLCYCLOHEXANE",
     choice="VDIWA2006.HEXYLCYCLOHEXANE",
     choice="VDIWA2006.CYCLOPENTENE",
     choice="VDIWA2006.CYCLOHEXENE",
     choice="VDIWA2006.BENZENE",
     choice="VDIWA2006.METHYLBENZENE",
     choice="VDIWA2006.ETHYLBENZENE",
     choice="VDIWA2006.PROPYLBENZENE",
     choice="VDIWA2006.BUTYLBENZENE",
     choice="VDIWA2006.PENTYLBENZENE",
     choice="VDIWA2006.HEXYLBENZENE",
     choice="VDIWA2006.O-XYLENE",
     choice="VDIWA2006.M-XYLENE",
     choice="VDIWA2006.P-XYLENE",
     choice="VDIWA2006.1,2,3-TRIMETHYLBENZENE",
     choice="VDIWA2006.1,2,4-TRIMETHYLBENZENE",
     choice="VDIWA2006.1,3,5-TRIMETHYLBENZENE",
     choice="VDIWA2006.1,2,3,4-TETRAMETHYLBENZENE",
     choice="VDIWA2006.1,2,3,5-TETRAMETHYLBENZENE",
     choice="VDIWA2006.1,2,4,5-TETRAMETHYLBENZENE",
     choice="VDIWA2006.PENTAMETHYLBENZENE",
     choice="VDIWA2006.HEXAMETHYLBENZENE",
     choice="VDIWA2006.STYRENE",
     choice="VDIWA2006.ISOPROPYLBENZENE",
     choice="VDIWA2006.BIPHENYL",
     choice="VDIWA2006.DIPHENYLMETHANE",
     choice="VDIWA2006.TRIPHENYLMETHANE",
     choice="VDIWA2006.TETRAPHENYLMETHANE",
     choice="VDIWA2006.NAPHTHALENE",
     choice="VDIWA2006.1-METHYLNAPHTHALENE",
     choice="VDIWA2006.2-METHYLNAPHTHALENE",
     choice="VDIWA2006.1-ETHYLNAPHTHALENE",
     choice="VDIWA2006.2-ETHYLNAPHTHALENE",
     choice="VDIWA2006.METHANOL",
     choice="VDIWA2006.ETHANOL",
     choice="VDIWA2006.PROPANOL",
     choice="VDIWA2006.BUTANOL",
     choice="VDIWA2006.PENTANOL",
     choice="VDIWA2006.HEXANOL",
     choice="VDIWA2006.HEPTANOL",
     choice="VDIWA2006.OCTANOL",
     choice="VDIWA2006.ISOPROPYL ALCOHOL",
     choice="VDIWA2006.2-METHYLPROPAN-1-OL",
     choice="VDIWA2006.ISOPENTYL ALCOHOL",
     choice="VDIWA2006.ETHYLENGLYCOL",
     choice="VDIWA2006.1,3-PROPYLENGLYCOL",
     choice="VDIWA2006.GLYCERIN",
     choice="VDIWA2006.CYCLOHEXANOL",
     choice="VDIWA2006.PHENYLMETHANOL",
     choice="VDIWA2006.O-CRESOL",
     choice="VDIWA2006.M-CRESOL",
     choice="VDIWA2006.P-CRESOL",
     choice="VDIWA2006.PHENOL",
     choice="VDIWA2006.METHANOIC ACID",
     choice="VDIWA2006.ETHANOIC ACID",
     choice="VDIWA2006.PROPANOIC ACID",
     choice="VDIWA2006.BUTANOIC ACID",
     choice="VDIWA2006.PENTANOIC ACID",
     choice="VDIWA2006.HEXANOIC ACID",
     choice="VDIWA2006.ACETIC ANHYDRIDE",
     choice="VDIWA2006.PROPANOYL PROPANOATE",
     choice="VDIWA2006.CHLOROACETIC ACID",
     choice="VDIWA2006.DICHLOROACETIC ACID",
     choice="VDIWA2006.TRICHLOROACETIC ACID",
     choice="VDIWA2006.CARBOMETHENE",
     choice="VDIWA2006.PROPANONE",
     choice="VDIWA2006.METHYL ETHYL KETONE",
     choice="VDIWA2006.DIETHYL KETONE",
     choice="VDIWA2006.PROPYL KETONE",
     choice="VDIWA2006.1-PHENYLETHANONE",
     choice="VDIWA2006.BENZOPHENONE",
     choice="VDIWA2006.METHOXYMETHANE",
     choice="VDIWA2006.ETHOXYETHANE",
     choice="VDIWA2006.PROPOXYPROPANE",
     choice="VDIWA2006.METHOXYPROPANE",
     choice="VDIWA2006.ETHOXYPROPANE",
     choice="VDIWA2006.ETHYLENE OXIDE",
     choice="VDIWA2006.FURAN",
     choice="VDIWA2006.1,4-DIOXANE",
     choice="VDIWA2006.METHANAL",
     choice="VDIWA2006.ACETALDEHYDE",
     choice="VDIWA2006.2,4,6-TRIMETHYL-1,3,5-TRIOXANE",
     choice="VDIWA2006.FURAN-2-CARBALDEHYDE",
     choice="VDIWA2006.BENZALDEHYDE",
     choice="VDIWA2006.2-HYDROXYBENZALDEHYDE",
     choice="VDIWA2006.METHYL-FORMATE",
     choice="VDIWA2006.ETHYL-FORMATE",
     choice="VDIWA2006.PROPYL-FORMATE",
     choice="VDIWA2006.METHYL-ACETATE",
     choice="VDIWA2006.ETHYL-ACETATE",
     choice="VDIWA2006.PROPYL-ACETATE",
     choice="VDIWA2006.METHYL-PROPIONATE",
     choice="VDIWA2006.ETHYL-PROPIONATE",
     choice="VDIWA2006.N-PROPYL-PROPIONATE",
     choice="VDIWA2006.METHYL-BUTANOATE",
     choice="VDIWA2006.ETHYL-BUTANOATE",
     choice="VDIWA2006.METHYL-BENZOATE",
     choice="VDIWA2006.ETHYL-BENZOATE",
     choice="VDIWA2006.METHYL-SALICYLATE",
     choice="VDIWA2006.METHYLAMINE",
     choice="VDIWA2006.ETHANAMINE",
     choice="VDIWA2006.PROPYLAMINE",
     choice="VDIWA2006.BUTAN-1-AMINE",
     choice="VDIWA2006.DIMETHYLAMINE",
     choice="VDIWA2006.TRIMETHYLAMINE",
     choice="VDIWA2006.DIETHYLAMINE",
     choice="VDIWA2006.TRIETHYLAMINE",
     choice="VDIWA2006.PIPERIDINE",
     choice="VDIWA2006.PYRIDINE",
     choice="VDIWA2006.PHENYLAMINE",
     choice="VDIWA2006.N-METHYL-ANILIN",
     choice="VDIWA2006.N,N-DIMETHYL-ANILIN",
     choice="VDIWA2006.N,N-DIETHYL-ANILIN",
     choice="VDIWA2006.PHENYLHYDRAZINE",
     choice="VDIWA2006.DIPHENYLAMINE",
     choice="VDIWA2006.ACETONITRILE",
     choice="VDIWA2006.PROPANENITRILE",
     choice="VDIWA2006.BUTANENITRILE",
     choice="VDIWA2006.BENZONITRILE",
     choice="VDIWA2006.METHANAMIDE",
     choice="VDIWA2006.NITROMETHANE",
     choice="VDIWA2006.NITROBENZENE",
     choice="VDIWA2006.O-NITROTOLUENE",
     choice="VDIWA2006.M-NITROTOLUENE",
     choice="VDIWA2006.P-NITROTOLUENE"));

    end LiquidName;

    type VLEFluidName "VLE Fluid name"
      extends String;
      annotation(choices(
          choice="TILMedia.CO2",
          choice="TILMedia.GERGCO2",
          choice="TILMedia.SPANCO2",
          choice="TILMedia.R134A",
          choice="TILMedia.ASTINASATOR134A",
          choice="TILMedia.R113",
          choice="TILMedia.R116",
          choice="TILMedia.R12",
          choice="TILMedia.R1234YF",
          choice="TILMedia.R1234ZEZ",
          choice="TILMedia.R1234ZE",
          choice="TILMedia.R124",
          choice="TILMedia.R125",
          choice="TILMedia.R141B",
          choice="TILMedia.R142B",
          choice="TILMedia.R143A",
          choice="TILMedia.R161",
          choice="TILMedia.R218",
          choice="TILMedia.R227EA",
          choice="TILMedia.R23",
          choice="TILMedia.R245FA",
          choice="TILMedia.R32",
          choice="TILMedia.R365MFC",
          choice="TILMedia.R404APPF",
          choice="TILMedia.R407CPPF",
          choice="TILMedia.R410APPF",
          choice="TILMedia.R507APPF",
          choice="TILMedia.RC318",
          choice="TILMedia.AMMONIA",
          choice="TILMedia.ARGON",
          choice="TILMedia.1-BUTENE",
          choice="TILMedia.CARBONDIOXIDE",
          choice="TILMedia.CARBONYLSULFIDE",
          choice="TILMedia.D4",
          choice="TILMedia.D5",
          choice="TILMedia.DEUTERIUM",
          choice="TILMedia.DIMETHYLCARBONATE",
          choice="TILMedia.DIMETHYLETHER",
          choice="TILMedia.ETHANE",
          choice="TILMedia.ETHANOL",
          choice="TILMedia.ETHYLBENZENE",
          choice="TILMedia.ETHYLENE",
          choice="TILMedia.HEAVYWATER",
          choice="TILMedia.HELIUM",
          choice="TILMedia.HYDROGENSULFIDE",
          choice="TILMedia.ISOBUTANE",
          choice="TILMedia.ISOPENTANE",
          choice="TILMedia.KRYPTON",
          choice="TILMedia.MD4M",
          choice="TILMedia.METHANE",
          choice="TILMedia.METHYLLINOLEATE",
          choice="TILMedia.METHYLOLEATE",
          choice="TILMedia.METHYLPALMITATE",
          choice="TILMedia.METHYLSTEARATE",
          choice="TILMedia.N-BUTANE",
          choice="TILMedia.N-DODECANE",
          choice="TILMedia.N-NONANE",
          choice="TILMedia.N-PROPANE",
          choice="TILMedia.NEON",
          choice="TILMedia.NEOPENTANE",
          choice="TILMedia.NITROGEN",
          choice="TILMedia.NITROUSOXIDE",
          choice="TILMedia.OXYGEN",
          choice="TILMedia.PARAHYDROGEN",
          choice="TILMedia.PROPANE",
          choice="TILMedia.PROPYLENE",
          choice="TILMedia.SULFURHEXAFLUORIDE",
          choice="TILMedia.TOLUENE",
          choice="TILMedia.WATER",
          choice="TILMedia.XENON",
          choice="TILMedia.O-XYLENE",
          choice="TILMedia.M-XYLENE",
          choice="TILMedia.P-XYLENE",
       choice="TILMediaRT.CO2",
       choice="TILMediaRT.R1234YF",
       choice="TILMediaRT.R134A",
       choice="TILMediaRT.R407C",
       choice="TILMediaRT.R410A",
       choice="TILMediaRT.WATER",
     choice="Refprop.CO2",
     choice="Refprop.R134A",
     choice="Refprop.1BUTENE.FLD",
     choice="Refprop.ACETONE.FLD",
     choice="Refprop.AIR.MIX",
     choice="Refprop.AIR.PPF",
     choice="Refprop.AMARILLO.MIX",
     choice="Refprop.AMMONIA.FLD",
     choice="Refprop.ARGON.FLD",
     choice="Refprop.BENZENE.FLD",
     choice="Refprop.BUTANE.FLD",
     choice="Refprop.C11.FLD",
     choice="Refprop.C12.FLD",
     choice="Refprop.C1CC6.FLD",
     choice="Refprop.C2BUTENE.FLD",
     choice="Refprop.C3CC6.FLD",
     choice="Refprop.C4F10.FLD",
     choice="Refprop.C5F12.FLD",
     choice="Refprop.CF3I.FLD",
     choice="Refprop.CO.FLD",
     choice="Refprop.CO2.FLD",
     choice="Refprop.COS.FLD",
     choice="Refprop.CYCLOHEX.FLD",
     choice="Refprop.CYCLOPEN.FLD",
     choice="Refprop.CYCLOPRO.FLD",
     choice="Refprop.D2.FLD",
     choice="Refprop.D2O.FLD",
     choice="Refprop.D4.FLD",
     choice="Refprop.D5.FLD",
     choice="Refprop.D6.FLD",
     choice="Refprop.DECANE.FLD",
     choice="Refprop.DEE.FLD",
     choice="Refprop.DMC.FLD",
     choice="Refprop.DME.FLD",
     choice="Refprop.EBENZENE.FLD",
     choice="Refprop.EKOFISK.MIX",
     choice="Refprop.ETHANE.FLD",
     choice="Refprop.ETHANOL.FLD",
     choice="Refprop.ETHYLENE.FLD",
     choice="Refprop.FLUORINE.FLD",
     choice="Refprop.GLFCOAST.MIX",
     choice="Refprop.H2S.FLD",
     choice="Refprop.HCL.FLD",
     choice="Refprop.HELIUM.FLD",
     choice="Refprop.HEPTANE.FLD",
     choice="Refprop.HEXANE.FLD",
     choice="Refprop.HIGHCO2.MIX",
     choice="Refprop.HIGHN2.MIX",
     choice="Refprop.HYDROGEN.FLD",
     choice="Refprop.IBUTENE.FLD",
     choice="Refprop.IHEXANE.FLD",
     choice="Refprop.IOCTANE.FLD",
     choice="Refprop.IPENTANE.FLD",
     choice="Refprop.ISOBUTAN.FLD",
     choice="Refprop.KRYPTON.FLD",
     choice="Refprop.MD2M.FLD",
     choice="Refprop.MD3M.FLD",
     choice="Refprop.MD4M.FLD",
     choice="Refprop.MDM.FLD",
     choice="Refprop.METHANE.FLD",
     choice="Refprop.METHANOL.FLD",
     choice="Refprop.MLINOLEA.FLD",
     choice="Refprop.MLINOLEN.FLD",
     choice="Refprop.MM.FLD",
     choice="Refprop.MOLEATE.FLD",
     choice="Refprop.MPALMITA.FLD",
     choice="Refprop.MSTEARAT.FLD",
     choice="Refprop.MXYLENE.FLD",
     choice="Refprop.N2O.FLD",
     choice="Refprop.NEON.FLD",
     choice="Refprop.NEOPENTN.FLD",
     choice="Refprop.NF3.FLD",
     choice="Refprop.NGSAMPLE.MIX",
     choice="Refprop.NITROGEN.FLD",
     choice="Refprop.NONANE.FLD",
     choice="Refprop.Novec 7000.FLD",
     choice="Refprop.NOVEC649.FLD",
     choice="Refprop.OCTANE.FLD",
     choice="Refprop.ORTHOHYD.FLD",
     choice="Refprop.OXYGEN.FLD",
     choice="Refprop.OXYLENE.FLD",
     choice="Refprop.PARAHYD.FLD",
     choice="Refprop.PENTANE.FLD",
     choice="Refprop.PROPANE.FLD",
     choice="Refprop.PROPYLEN.FLD",
     choice="Refprop.PROPYNE.FLD",
     choice="Refprop.PXYLENE.FLD",
     choice="Refprop.R11.FLD",
     choice="Refprop.R113.FLD",
     choice="Refprop.R114.FLD",
     choice="Refprop.R115.FLD",
     choice="Refprop.R116.FLD",
     choice="Refprop.R12.FLD",
     choice="Refprop.R1216.FLD",
     choice="Refprop.R123.FLD",
     choice="Refprop.R1233ZD.FLD",
     choice="Refprop.R1234YF.FLD",
     choice="Refprop.R1234ZE.FLD",
     choice="Refprop.R124.FLD",
     choice="Refprop.R125.FLD",
     choice="Refprop.R13.FLD",
     choice="Refprop.R134A.FLD",
     choice="Refprop.R14.FLD",
     choice="Refprop.R141B.FLD",
     choice="Refprop.R142B.FLD",
     choice="Refprop.R143A.FLD",
     choice="Refprop.R152A.FLD",
     choice="Refprop.R161.FLD",
     choice="Refprop.R21.FLD",
     choice="Refprop.R218.FLD",
     choice="Refprop.R22.FLD",
     choice="Refprop.R227EA.FLD",
     choice="Refprop.R23.FLD",
     choice="Refprop.R236EA.FLD",
     choice="Refprop.R236FA.FLD",
     choice="Refprop.R245CA.FLD",
     choice="Refprop.R245FA.FLD",
     choice="Refprop.R32.FLD",
     choice="Refprop.R365MFC.FLD",
     choice="Refprop.R40.FLD",
     choice="Refprop.R401A.MIX",
     choice="Refprop.R401B.MIX",
     choice="Refprop.R401C.MIX",
     choice="Refprop.R402A.MIX",
     choice="Refprop.R402B.MIX",
     choice="Refprop.R403A.MIX",
     choice="Refprop.R403B.MIX",
     choice="Refprop.R404A.MIX",
     choice="Refprop.R404A.PPF",
     choice="Refprop.R405A.MIX",
     choice="Refprop.R406A.MIX",
     choice="Refprop.R407A.MIX",
     choice="Refprop.R407B.MIX",
     choice="Refprop.R407C.MIX",
     choice="Refprop.R407C.PPF",
     choice="Refprop.R407D.MIX",
     choice="Refprop.R407E.MIX",
     choice="Refprop.R407F.MIX",
     choice="Refprop.R408A.MIX",
     choice="Refprop.R409A.MIX",
     choice="Refprop.R409B.MIX",
     choice="Refprop.R41.FLD",
     choice="Refprop.R410A.MIX",
     choice="Refprop.R410A.PPF",
     choice="Refprop.R410B.MIX",
     choice="Refprop.R411A.MIX",
     choice="Refprop.R411B.MIX",
     choice="Refprop.R412A.MIX",
     choice="Refprop.R413A.MIX",
     choice="Refprop.R414A.MIX",
     choice="Refprop.R414B.MIX",
     choice="Refprop.R415A.MIX",
     choice="Refprop.R415B.MIX",
     choice="Refprop.R416A.MIX",
     choice="Refprop.R417A.MIX",
     choice="Refprop.R418A.MIX",
     choice="Refprop.R419A.MIX",
     choice="Refprop.R420A.MIX",
     choice="Refprop.R421A.MIX",
     choice="Refprop.R421B.MIX",
     choice="Refprop.R422A.MIX",
     choice="Refprop.R422B.MIX",
     choice="Refprop.R422C.MIX",
     choice="Refprop.R422D.MIX",
     choice="Refprop.R423A.MIX",
     choice="Refprop.R424A.MIX",
     choice="Refprop.R425A.MIX",
     choice="Refprop.R426A.MIX",
     choice="Refprop.R427A.MIX",
     choice="Refprop.R428A.MIX",
     choice="Refprop.R429A.MIX",
     choice="Refprop.R430A.MIX",
     choice="Refprop.R431A.MIX",
     choice="Refprop.R432A.MIX",
     choice="Refprop.R433A.MIX",
     choice="Refprop.R434A.MIX",
     choice="Refprop.R435A.MIX",
     choice="Refprop.R436A.MIX",
     choice="Refprop.R436B.MIX",
     choice="Refprop.R437A.MIX",
     choice="Refprop.R438A.MIX",
     choice="Refprop.R441A.MIX",
     choice="Refprop.R442A.MIX",
     choice="Refprop.R443A.MIX",
     choice="Refprop.R444A.MIX",
     choice="Refprop.R500.MIX",
     choice="Refprop.R501.MIX",
     choice="Refprop.R502.MIX",
     choice="Refprop.R503.MIX",
     choice="Refprop.R504.MIX",
     choice="Refprop.R507A.MIX",
     choice="Refprop.R507A.PPF",
     choice="Refprop.R508A.MIX",
     choice="Refprop.R508B.MIX",
     choice="Refprop.R509A.MIX",
     choice="Refprop.R510A.MIX",
     choice="Refprop.R512A.MIX",
     choice="Refprop.RC318.FLD",
     choice="Refprop.RE143A.FLD",
     choice="Refprop.RE245CB2.FLD",
     choice="Refprop.RE245FA2.FLD",
     choice="Refprop.RE347MCC.FLD",
     choice="Refprop.SF6.FLD",
     choice="Refprop.SO2.FLD",
     choice="Refprop.T2BUTENE.FLD",
     choice="Refprop.TOLUENE.FLD",
     choice="Refprop.WATER.FLD",
     choice="Refprop.XENON.FLD"));
    end VLEFluidName;

    type SLEMediumName "SLEMaterial name"
    extends String;

    annotation(choices(
      choice="SLEMedium.SimpleAdBlue",
      choice="SLEMedium.AdBlue",
      choice="SLEMedium.SimpleWater"));
    end SLEMediumName;

    record CriticalDataRecord "Critical data record"
       extends TILMedia.Internals.ClassTypes.Record;

      Modelica.SIunits.Density d "Critical density";
      Modelica.SIunits.SpecificEnthalpy h "Critical specific enthalpy";
      Modelica.SIunits.AbsolutePressure p "Critical pressure";
      Modelica.SIunits.SpecificEntropy s "Critical specific entropy";
      Modelica.SIunits.Temperature T "Critical temperature";
      annotation(defaultComponentName="crit");
    end CriticalDataRecord;

    record PropertyRecord "Property record"
      extends TILMedia.Internals.ClassTypes.Record;
      SI.Density d "Density";
      SI.SpecificEnthalpy h "Specific enthalpy";
      SI.AbsolutePressure p "Pressure";
      SI.SpecificEntropy s "Specific entropy";
      SI.Temperature T "Temperature";
      SI.MassFraction q=0 "Steam mass fraction (quality)";
      SI.SpecificHeatCapacity cp=0 "Specific isobaric heat capacity cp";

      TILMedia.Internals.CriticalDataRecord crit=
               TILMedia.Internals.CriticalDataRecord(d=0.0,T=0.0,p=0.0,h=0.0,s=0.0)
        "Critical data record";
      TILMedia.Internals.VLERecordSimple VLE=
               TILMedia.Internals.VLERecordSimple(d_l=0.0, h_l=0.0, p_l=0.0, s_l=0.0, T_l=0.0, d_v=0.0, h_v=0.0, p_v=0.0, s_v=0.0, T_v=0.0)
        "Saturation property record";
      TILMedia.Internals.VLETransportPropertyRecord VLETransp=
               TILMedia.Internals.VLETransportPropertyRecord(Pr_l=0.0, Pr_v=0.0, eta_l=0.0, eta_v=0.0, lambda_l=0.0, lambda_v=0.0)
        "Saturation property record";
      TILMedia.Internals.TransportPropertyRecord transp=
               TILMedia.Internals.TransportPropertyRecord(Pr=0.0,lambda=0.0,eta=0.0,sigma=0.0)
        "Transport property record";

      annotation(defaultComponentName="properties");
    end PropertyRecord;

    record VLERecord "VLE property record"
      extends TILMedia.Internals.ClassTypes.Record;
      SI.Density d_l "Density of liquid phase";
      SI.Density d_v "Density of vapour phase";
      SI.SpecificEnthalpy h_l "Specific enthalpy of liquid phase";
      SI.SpecificEnthalpy h_v "Specific enthalpy of vapour phase";
      SI.AbsolutePressure p_l "Pressure of liquid phase";
      SI.AbsolutePressure p_v "Pressure of vapour phase";
      SI.SpecificEntropy s_l "Specific entropy of liquid phase";
      SI.SpecificEntropy s_v "Specific entropy of vapour phase";
      SI.Temperature T_l "Temperature of liquid phase";
      SI.Temperature T_v "Temperature of vapour phase";
      SI.MassFraction[nc-1] xi_l "Mass fraction of liquid phase";
      SI.MassFraction[nc-1] xi_v "Mass fraction of vapour phase";
      parameter Integer nc;
    end VLERecord;

    record VLERecordSimple "VLE property record"
      extends TILMedia.Internals.ClassTypes.Record;
      SI.Density d_l "Density of liquid phase";
      SI.Density d_v "Density of vapour phase";
      SI.SpecificEnthalpy h_l "Specific enthalpy of liquid phase";
      SI.SpecificEnthalpy h_v "Specific enthalpy of vapour phase";
      SI.AbsolutePressure p_l "Pressure of liquid phase";
      SI.AbsolutePressure p_v "Pressure of vapour phase";
      SI.SpecificEntropy s_l "Specific entropy of liquid phase";
      SI.SpecificEntropy s_v "Specific entropy of vapour phase";
      SI.Temperature T_l "Temperature of liquid phase";
      SI.Temperature T_v "Temperature of vapour phase";
    end VLERecordSimple;

    record AdditionalVLERecord "Additional VLE property record"
       extends TILMedia.Internals.ClassTypes.Record;
      Modelica.SIunits.SpecificHeatCapacity cp_l
        "Specific heat capacity cp of liquid phase";
      Modelica.SIunits.SpecificHeatCapacity cp_v
        "Specific heat capacity cp of vapour phase";
      Modelica.SIunits.LinearExpansionCoefficient beta_l
        "Isobaric expansion coefficient of liquid phase";
      Modelica.SIunits.LinearExpansionCoefficient beta_v
        "Isobaric expansion coefficient of vapour phase";
      Modelica.SIunits.Compressibility kappa_l
        "Isothermal compressibility of liquid phase";
      Modelica.SIunits.Compressibility kappa_v
        "Isothermal compressibility of vapour phase";
    end AdditionalVLERecord;

    record VLETransportPropertyRecord "Transport property record"
       extends TILMedia.Internals.ClassTypes.Record;
      SI.PrandtlNumber Pr_l "Prandtl number of liquid phase";
      SI.PrandtlNumber Pr_v "Prandtl number of vapour phase";
      SI.ThermalConductivity lambda_l "Thermal conductivity of liquid phase";
      SI.ThermalConductivity lambda_v "Thermal conductivity of vapour phase";
      SI.DynamicViscosity eta_l(min=-1) "Dynamic viscosity of liquid phase";
      SI.DynamicViscosity eta_v(min=-1) "Dynamic viscosity of vapour phase";

      annotation(defaultComponentName="transp");
    end VLETransportPropertyRecord;

    record SolidPropertyRecord
       extends TILMedia.Internals.ClassTypes.Record;

      Modelica.SIunits.Density d "Density";
      Modelica.SIunits.Temperature T "Temperature";
      Modelica.SIunits.SpecificHeatCapacity cp "Heat capacity";
      Modelica.SIunits.ThermalConductivity lambda "Thermal conductivity";
    end SolidPropertyRecord;

    record TransportPropertyRecord "Transport property record"
       extends TILMedia.Internals.ClassTypes.Record;
      Modelica.SIunits.PrandtlNumber Pr "Prandtl number";
      Modelica.SIunits.ThermalConductivity lambda "Thermal conductivity";
      Modelica.SIunits.DynamicViscosity eta(min=-1) "Dynamic viscosity";
      Modelica.SIunits.SurfaceTension sigma "Surface tension";
      annotation(defaultComponentName="transp");
    end TransportPropertyRecord;

    function calcComputeFlags
      input Boolean computeTransportProperties;
      input Boolean interpolateTransportProperties;
      input Boolean computeSurfaceTension;
      input Boolean deactivateTwoPhaseRegion;
      output Integer flags;
    algorithm
      flags := array(1,2,4,8)*array(if (computeTransportProperties) then 1 else 0,if (interpolateTransportProperties) then 1 else 0,if (computeSurfaceTension) then 1 else 0,if (deactivateTwoPhaseRegion) then 1 else 0);
    //  flags := 0;
    /*  if computeTransportProperties then
    flags := flags + 1;
  end if;
  if interpolateTransportProperties then
    flags := flags + 2;
  end if;
  if computeSurfaceTension then
    flags := flags + 4;
  end if;*/

      annotation(Inline=true);
    end calcComputeFlags;

    function redirectModelicaFormatMessage
      input Real y=0;
      //protected
      output Integer x;
    external"C" x = TILMedia_redirectModelicaFormatMessage_wrapper() annotation (
          Include="
#ifndef TILMEDIAMODELICAFORMATMESSAGE
#define TILMEDIAMODELICAFORMATMESSAGE
#ifdef DYMOLA_STATIC
int TILMedia_redirectModelicaFormatMessage(void* _str);
int TILMedia_redirectModelicaFormatError(void* _str);
int TILMedia_redirectDymolaErrorFunction(void* _str);
#ifndef _WIN32
#define __stdcall
#endif
double __stdcall TILMedia_DymosimErrorLevWrapper(const char* message, int level) {
    return DymosimErrorLev(message, level);
};

int TILMedia_redirectModelicaFormatMessage_wrapper(){
  TILMedia_redirectModelicaFormatMessage((void*)ModelicaFormatMessage);
  TILMedia_redirectModelicaFormatError((void*)ModelicaFormatError);
  TILMedia_redirectDymolaErrorFunction((void*)TILMedia_DymosimErrorLevWrapper);
  return 0;
}
#endif
#endif
",    Library="TILMedia341");
    end redirectModelicaFormatMessage;

    function concatNames
     input String names[:];
     output String concatName;
    algorithm
      concatName := "";
      if (size(names, 1)>0) then
      concatName := names[1];
      end if;

        for i in 2:size(names, 1) loop
          concatName := concatName + "|" + names[i];
        end for;
    end concatNames;

    function getPropertiesVLE
      input Real d,  h,  p,  s,  T,  cp,  q;
      input Real d_crit,  h_crit,  p_crit,  s_crit,  T_crit;
      input Real d_l,  h_l,  p_l,  s_l,  T_l,  d_v,  h_v,  p_v,  s_v,  T_v;
      input Real Pr,  lambda,  eta,  sigma;
      input Real Pr_l,  Pr_v,  lambda_l,  lambda_v,  eta_l,  eta_v;
      output TILMedia.Internals.PropertyRecord properties;
    algorithm
      properties.d := d;
      properties.h := h;
      properties.p := p;
      properties.s := s;
      properties.T := T;
      properties.cp := cp;
      properties.q := q;
      properties.VLE := TILMedia.Internals.VLERecordSimple(
        d_l=d_l,
        h_l=h_l,
        p_l=p_l,
        s_l=s_l,
        T_l=T_l,
        d_v=d_v,
        h_v=h_v,
        p_v=p_v,
        s_v=s_v,
        T_v=T_v);
      properties.VLETransp := TILMedia.Internals.VLETransportPropertyRecord(
        Pr_l=Pr_l,
        Pr_v=Pr_v,
        lambda_l=lambda_l,
        lambda_v=lambda_v,
        eta_l=eta_l,
        eta_v=eta_v);
      properties.transp := TILMedia.Internals.TransportPropertyRecord(
        Pr=Pr,
        lambda=lambda,
        eta=eta,
        sigma=sigma);
      properties.crit := TILMedia.Internals.CriticalDataRecord(
        d=d_crit,
        h=h_crit,
        p=p_crit,
        s=s_crit,
        T=T_crit);
    end getPropertiesVLE;
   annotation (Icon(
        graphics={
          Rectangle(
            extent={{-100,-100},{80,50}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-100,50},{-80,70},{100,70},{80,50},{-100,50}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{100,70},{100,-80},{80,-100},{80,50},{100,70}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end Internals;
annotation (
preferedView="info",
version="3.4.1",
  uses(Modelica(version="3.2.2")),
Documentation(info="<html>
<br>
<img src=../Images/infoTILMedia.png><br>
<br>
  <hr>
</html>"),                 Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={1,1}), graphics={Text(
          extent={{-120,143},{120,94}},
          lineColor={255,0,0},
          textString=
               "%name"), Bitmap(
        extent={{-100,-100},{100,100}},
        imageSource=
            "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",
        fileName="modelica://TILMedia/Images/TILMedia.png")}),
            conversion(
    from(version="2.0.4"),
    from(version="2.1.0", script="modelica://TILMedia/Scripts/ConvertTILMedia_to_3.0.0.mos"),
    from(version="2.1.1", script="modelica://TILMedia/Scripts/ConvertTILMedia_to_3.0.0.mos"),
    from(version="2.1.2", script="modelica://TILMedia/Scripts/ConvertTILMedia_to_3.0.0.mos"),
    from(version="3.0.0", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.1andEarlier.mos"),
    from(version="3.0.1", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.1andEarlier.mos"),
    from(version="3.0.2", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.1andEarlier.mos"),
    from(version="3.1.0", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.1andEarlier.mos"),
    from(version="3.2.0", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.1andEarlier.mos"),
    from(version="3.2.1", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.1andEarlier.mos"),
    from(version="3.2.2", script="modelica://TILMedia/Scripts/ConvertTILMedia_from_3.2.2.mos")));
end TILMedia;

package Modelica "Modelica Standard Library - Version 3.2.2"
extends Modelica.Icons.Package;

  package Blocks
  "Library of basic input/output control blocks (continuous, discrete, logical, table blocks)"
  import SI = Modelica.SIunits;
  extends Modelica.Icons.Package;

    package Continuous
    "Library of continuous control blocks with internal states"
      import Modelica.Blocks.Interfaces;
      import Modelica.SIunits;
      extends Modelica.Icons.Package;

      block FirstOrder "First order transfer function block (= 1 pole)"
        import Modelica.Blocks.Types.Init;
        parameter Real k(unit="1")=1 "Gain";
        parameter SIunits.Time T(start=1) "Time Constant";
        parameter Modelica.Blocks.Types.Init initType=Modelica.Blocks.Types.Init.NoInit
          "Type of initialization (1: no init, 2: steady state, 3/4: initial output)"     annotation(Evaluate=true,
            Dialog(group="Initialization"));
        parameter Real y_start=0 "Initial or guess value of output (= state)"
          annotation (Dialog(group="Initialization"));

        extends Interfaces.SISO(y(start=y_start));

      initial equation
        if initType == Init.SteadyState then
          der(y) = 0;
        elseif initType == Init.InitialState or initType == Init.InitialOutput then
          y = y_start;
        end if;
      equation
        der(y) = (k*u - y)/T;
        annotation (
          Documentation(info="<html>
<p>
This blocks defines the transfer function between the input u
and the output y (element-wise) as <i>first order</i> system:
</p>
<pre>
               k
     y = ------------ * u
            T * s + 1
</pre>
<p>
If you would like to be able to change easily between different
transfer functions (FirstOrder, SecondOrder, ... ) by changing
parameters, use the general block <b>TransferFunction</b> instead
and model a first order SISO system with parameters<br>
b = {k}, a = {T, 1}.
</p>
<pre>
Example:
   parameter: k = 0.3, T = 0.4
   results in:
             0.3
      y = ----------- * u
          0.4 s + 1.0
</pre>

</html>"),       Icon(
        coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{-80.0,78.0},{-80.0,-90.0}},
          color={192,192,192}),
        Polygon(lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
        Line(points={{-90.0,-80.0},{82.0,-80.0}},
          color={192,192,192}),
        Polygon(lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
        Line(origin = {-26.667,6.667},
            points = {{106.667,43.333},{-13.333,29.333},{-53.333,-86.667}},
            color = {0,0,127},
            smooth = Smooth.Bezier),
        Text(lineColor={192,192,192},
          extent={{0.0,-60.0},{60.0,0.0}},
          textString="PT1"),
        Text(extent={{-150.0,-150.0},{150.0,-110.0}},
          textString="T=%T")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Text(
                extent={{-48,52},{50,8}},
                lineColor={0,0,0},
                textString="k"),
              Text(
                extent={{-54,-6},{56,-56}},
                lineColor={0,0,0},
                textString="T s + 1"),
              Line(points={{-50,0},{50,0}}),
              Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,255}),
              Line(points={{-100,0},{-60,0}}, color={0,0,255}),
              Line(points={{60,0},{100,0}}, color={0,0,255})}));
      end FirstOrder;
      annotation (
        Documentation(info="<html>
<p>
This package contains basic <b>continuous</b> input/output blocks
described by differential equations.
</p>

<p>
All blocks of this package can be initialized in different
ways controlled by parameter <b>initType</b>. The possible
values of initType are defined in
<a href=\"modelica://Modelica.Blocks.Types.Init\">Modelica.Blocks.Types.Init</a>:
</p>

<table border=1 cellspacing=0 cellpadding=2>
  <tr><td valign=\"top\"><b>Name</b></td>
      <td valign=\"top\"><b>Description</b></td></tr>

  <tr><td valign=\"top\"><b>Init.NoInit</b></td>
      <td valign=\"top\">no initialization (start values are used as guess values with fixed=false)</td></tr>

  <tr><td valign=\"top\"><b>Init.SteadyState</b></td>
      <td valign=\"top\">steady state initialization (derivatives of states are zero)</td></tr>

  <tr><td valign=\"top\"><b>Init.InitialState</b></td>
      <td valign=\"top\">Initialization with initial states</td></tr>

  <tr><td valign=\"top\"><b>Init.InitialOutput</b></td>
      <td valign=\"top\">Initialization with initial outputs (and steady state of the states if possible)</td></tr>
</table>

<p>
For backward compatibility reasons the default of all blocks is
<b>Init.NoInit</b>, with the exception of Integrator and LimIntegrator
where the default is <b>Init.InitialState</b> (this was the initialization
defined in version 2.2 of the Modelica standard library).
</p>

<p>
In many cases, the most useful initial condition is
<b>Init.SteadyState</b> because initial transients are then no longer
present. The drawback is that in combination with a non-linear
plant, non-linear algebraic equations occur that might be
difficult to solve if appropriate guess values for the
iteration variables are not provided (i.e., start values with fixed=false).
However, it is often already useful to just initialize
the linear blocks from the Continuous blocks library in SteadyState.
This is uncritical, because only linear algebraic equations occur.
If Init.NoInit is set, then the start values for the states are
interpreted as <b>guess</b> values and are propagated to the
states with fixed=<b>false</b>.
</p>

<p>
Note, initialization with Init.SteadyState is usually difficult
for a block that contains an integrator
(Integrator, LimIntegrator, PI, PID, LimPID).
This is due to the basic equation of an integrator:
</p>

<pre>
  <b>initial equation</b>
     <b>der</b>(y) = 0;   // Init.SteadyState
  <b>equation</b>
     <b>der</b>(y) = k*u;
</pre>

<p>
The steady state equation leads to the condition that the input to the
integrator is zero. If the input u is already (directly or indirectly) defined
by another initial condition, then the initialization problem is <b>singular</b>
(has none or infinitely many solutions). This situation occurs often
for mechanical systems, where, e.g., u = desiredSpeed - measuredSpeed and
since speed is both a state and a derivative, it is always defined by
Init.InitialState or Init.SteadyState initialization.
</p>

<p>
In such a case, <b>Init.NoInit</b> has to be selected for the integrator
and an additional initial equation has to be added to the system
to which the integrator is connected. E.g., useful initial conditions
for a 1-dim. rotational inertia controlled by a PI controller are that
<b>angle</b>, <b>speed</b>, and <b>acceleration</b> of the inertia are zero.
</p>

</html>"),     Icon(graphics={Line(
              origin={0.061,4.184},
              points={{81.939,36.056},{65.362,36.056},{14.39,-26.199},{-29.966,
                  113.485},{-65.374,-61.217},{-78.061,-78.184}},
              color={95,95,95},
              smooth=Smooth.Bezier)}));
    end Continuous;

    package Interfaces
    "Library of connectors and partial models for input/output blocks"
      import Modelica.SIunits;
      extends Modelica.Icons.InterfacesPackage;

      connector RealInput = input Real "'input Real' as connector" annotation (
        defaultComponentName="u",
        Icon(graphics={
          Polygon(
            lineColor={0,0,127},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid,
            points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
          coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
            preserveAspectRatio=true,
            initialScale=0.2)),
        Diagram(
          coordinateSystem(preserveAspectRatio=true,
            initialScale=0.2,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Polygon(
            lineColor={0,0,127},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid,
            points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
          Text(
            lineColor={0,0,127},
            extent={{-10.0,60.0},{-10.0,85.0}},
            textString="%name")}),
        Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));

      connector RealOutput = output Real "'output Real' as connector" annotation (
        defaultComponentName="y",
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Polygon(
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
        Diagram(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Polygon(
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            points={{-100.0,50.0},{0.0,0.0},{-100.0,-50.0}}),
          Text(
            lineColor={0,0,127},
            extent={{30.0,60.0},{30.0,110.0}},
            textString="%name")}),
        Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));

      connector BooleanInput = input Boolean "'input Boolean' as connector"
        annotation (
        defaultComponentName="u",
        Icon(graphics={Polygon(
              points={{-100,100},{100,0},{-100,-100},{-100,100}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid)}, coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=true,
            initialScale=0.2)),
        Diagram(coordinateSystem(
            preserveAspectRatio=true,
            initialScale=0.2,
            extent={{-100,-100},{100,100}}), graphics={Polygon(
              points={{0,50},{100,0},{0,-50},{0,50}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-10,85},{-10,60}},
              lineColor={255,0,255},
              textString="%name")}),
        Documentation(info="<html>
<p>
Connector with one input signal of type Boolean.
</p>
</html>"));

      connector BooleanOutput = output Boolean "'output Boolean' as connector"
        annotation (
        defaultComponentName="y",
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={Polygon(
              points={{-100,100},{100,0},{-100,-100},{-100,100}},
              lineColor={255,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={Polygon(
              points={{-100,50},{0,0},{-100,-50},{-100,50}},
              lineColor={255,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{30,110},{30,60}},
              lineColor={255,0,255},
              textString="%name")}),
        Documentation(info="<html>
<p>
Connector with one output signal of type Boolean.
</p>
</html>"));

      connector RealVectorInput = input Real
        "Real input connector used for vector of connectors" annotation (
        defaultComponentName="u",
        Icon(graphics={Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid)}, coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=true,
            initialScale=0.2)),
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            initialScale=0.2,
            extent={{-100,-100},{100,100}}), graphics={Text(
              extent={{-10,85},{-10,60}},
              lineColor={0,0,127},
              textString="%name"), Ellipse(
              extent={{-50,50},{50,-50}},
              lineColor={0,0,127},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
Real input connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Interfaces.PartialRealMISO\">PartialRealMISO</a>,
and has therefore a different icon as RealInput connector.
</p>
</html>"));

      partial block SO "Single Output continuous control block"
        extends Modelica.Blocks.Icons.Block;

        RealOutput y "Connector of Real output signal" annotation (Placement(
              transformation(extent={{100,-10},{120,10}})));
        annotation (Documentation(info="<html>
<p>
Block has one continuous Real output signal.
</p>
</html>"));

      end SO;

      partial block SISO "Single Input Single Output continuous control block"
        extends Modelica.Blocks.Icons.Block;

        RealInput u "Connector of Real input signal" annotation (Placement(
              transformation(extent={{-140,-20},{-100,20}})));
        RealOutput y "Connector of Real output signal" annotation (Placement(
              transformation(extent={{100,-10},{120,10}})));
        annotation (Documentation(info="<html>
<p>
Block has one continuous Real input and one continuous Real output signal.
</p>
</html>"));
      end SISO;

      partial block SI2SO
        "2 Single Input / 1 Single Output continuous control block"
        extends Modelica.Blocks.Icons.Block;

        RealInput u1 "Connector of Real input signal 1" annotation (Placement(
              transformation(extent={{-140,40},{-100,80}})));
        RealInput u2 "Connector of Real input signal 2" annotation (Placement(
              transformation(extent={{-140,-80},{-100,-40}})));
        RealOutput y "Connector of Real output signal" annotation (Placement(
              transformation(extent={{100,-10},{120,10}})));

        annotation (Documentation(info="<html>
<p>
Block has two continuous Real input signals u1 and u2 and one
continuous Real output signal y.
</p>
</html>"));

      end SI2SO;

      partial block PartialRealMISO
        "Partial block with a RealVectorInput and a RealOutput signal"

        parameter Integer significantDigits(min=1) = 3
          "Number of significant digits to be shown in dynamic diagram layer for y"
          annotation (Dialog(tab="Advanced"));
        parameter Integer nu(min=0) = 0 "Number of input connections"
          annotation (Dialog(connectorSizing=true), HideResult=true);
        Modelica.Blocks.Interfaces.RealVectorInput u[nu]
          annotation (Placement(transformation(extent={{-120,70},{-80,-70}})));
        Modelica.Blocks.Interfaces.RealOutput y
          annotation (Placement(transformation(extent={{100,-17},{134,17}})));
        annotation (Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}},
              initialScale=0.06), graphics={
              Text(
                extent={{110,-50},{300,-70}},
                lineColor={0,0,0},
                textString=DynamicSelect(" ", String(y, significantDigits=
                    significantDigits))),
              Text(
                extent={{-250,170},{250,110}},
                textString="%name",
                lineColor={0,0,255}),
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={255,137,0},
                lineThickness=5.0,
                fillColor={255,255,255},
                borderPattern=BorderPattern.Raised,
                fillPattern=FillPattern.Solid)}));
      end PartialRealMISO;

      partial block SignalSource "Base class for continuous signal source"
        extends SO;
        parameter Real offset=0 "Offset of output signal y";
        parameter SIunits.Time startTime=0 "Output y = offset for time < startTime";
        annotation (Documentation(info="<html>
<p>
Basic block for Real sources of package Blocks.Sources.
This component has one continuous Real output signal y
and two parameters (offset, startTime) to shift the
generated signal.
</p>
</html>"));
      end SignalSource;

      partial block BooleanSISO
        "Single Input Single Output control block with signals of type Boolean"

        extends Modelica.Blocks.Icons.BooleanBlock;

      public
        BooleanInput u "Connector of Boolean input signal" annotation (Placement(
              transformation(extent={{-140,-20},{-100,20}})));
        BooleanOutput y "Connector of Boolean output signal" annotation (Placement(
              transformation(extent={{100,-10},{120,10}})));

        annotation (Documentation(info="<html>
<p>
Block has one continuous Boolean input and one continuous Boolean output signal.
</p>
</html>"));
      end BooleanSISO;

      partial block partialBooleanSource
        "Partial source block (has 1 output Boolean signal and an appropriate default icon)"
        extends Modelica.Blocks.Icons.PartialBooleanBlock;

        Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));

        annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}), graphics={
              Polygon(
                points={{-80,88},{-88,66},{-72,66},{-80,88}},
                lineColor={255,0,255},
                fillColor={255,0,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,66},{-80,-82}}, color={255,0,255}),
              Line(points={{-90,-70},{72,-70}}, color={255,0,255}),
              Polygon(
                points={{90,-70},{68,-62},{68,-78},{90,-70}},
                lineColor={255,0,255},
                fillColor={255,0,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{71,7},{85,-7}},
                lineColor=DynamicSelect({235,235,235}, if y > 0.5 then {0,255,0}
                     else {235,235,235}),
                fillColor=DynamicSelect({235,235,235}, if y > 0.5 then {0,255,0}
                     else {235,235,235}),
                fillPattern=FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Polygon(
                  points={{-70,92},{-76,70},{-64,70},{-70,92}},
                  lineColor={95,95,95},
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid),Line(points={{-70,70},{-70,-88}},
                color={95,95,95}),Line(points={{-90,-70},{68,-70}}, color={95,95,95}),
                Polygon(
                  points={{90,-70},{68,-64},{68,-76},{90,-70}},
                  lineColor={95,95,95},
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{54,-80},{106,-92}},
                  lineColor={0,0,0},
                  textString="time"),Text(
                  extent={{-64,92},{-46,74}},
                  lineColor={0,0,0},
                  textString="y")}),
          Documentation(info="<html>
<p>
Basic block for Boolean sources of package Blocks.Sources.
This component has one continuous Boolean output signal y
and a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

      end partialBooleanSource;

      partial block BlockIcon
        "This icon will be removed in future Modelica versions, use Modelica.Blocks.Icons.Block instead."
        // extends Modelica.Icons.ObsoleteModel;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Rectangle(
                extent={{-100,-100},{100,100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(info="<html>
<p>
This icon will be removed in future versions of the Modelica Standard Library.
Instead the icon <a href=\"modelica://Modelica.Blocks.Icons.Block\">Modelica.Blocks.Icons.Block</a> shall be used.
</p>
</html>"));

      end BlockIcon;
      annotation (Documentation(info="<html>
<p>
This package contains interface definitions for
<b>continuous</b> input/output blocks with Real,
Integer and Boolean signals. Furthermore, it contains
partial models for continuous and discrete blocks.
</p>

</html>",     revisions="<html>
<ul>
<li><i>Oct. 21, 2002</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Added several new interfaces.</li>
<li><i>Oct. 24, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       RealInputSignal renamed to RealInput. RealOutputSignal renamed to
       output RealOutput. GraphBlock renamed to BlockIcon. SISOreal renamed to
       SISO. SOreal renamed to SO. I2SOreal renamed to M2SO.
       SignalGenerator renamed to SignalSource. Introduced the following
       new models: MIMO, MIMOs, SVcontrol, MVcontrol, DiscreteBlockIcon,
       DiscreteBlock, DiscreteSISO, DiscreteMIMO, DiscreteMIMOs,
       BooleanBlockIcon, BooleanSISO, BooleanSignalSource, MI2BooleanMOs.</li>
<li><i>June 30, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
    end Interfaces;

    package Math
    "Library of Real mathematical functions as input/output blocks"
      import Modelica.SIunits;
      import Modelica.Blocks.Interfaces;
      extends Modelica.Icons.Package;

      block Gain "Output the product of a gain value with the input signal"

        parameter Real k(start=1, unit="1")
          "Gain value multiplied with input signal";
      public
        Interfaces.RealInput u "Input signal connector" annotation (Placement(
              transformation(extent={{-140,-20},{-100,20}})));
        Interfaces.RealOutput y "Output signal connector" annotation (Placement(
              transformation(extent={{100,-10},{120,10}})));

      equation
        y = k*u;
        annotation (
          Documentation(info="<html>
<p>
This block computes output <i>y</i> as
<i>product</i> of gain <i>k</i> with the
input <i>u</i>:
</p>
<pre>
    y = k * u;
</pre>

</html>"),Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}), graphics={
              Polygon(
                points={{-100,-100},{-100,100},{100,0},{-100,-100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-150,-140},{150,-100}},
                lineColor={0,0,0},
                textString="k=%k"),
              Text(
                extent={{-150,140},{150,100}},
                textString="%name",
                lineColor={0,0,255})}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Polygon(
                  points={{-100,-100},{-100,100},{100,0},{-100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-76,38},{0,-34}},
                  textString="k",
                  lineColor={0,0,255})}));
      end Gain;

      block MultiSum "Sum of Reals: y = k[1]*u[1] + k[2]*u[2] + ... + k[n]*u[n]"
        extends Modelica.Blocks.Interfaces.PartialRealMISO;
        parameter Real k[nu]=fill(1, nu) "Input gains";
      equation
        if size(u, 1) > 0 then
          y = k*u;
        else
          y = 0;
        end if;

        annotation (Icon(graphics={Text(
                extent={{-200,-110},{200,-140}},
                lineColor={0,0,0},
                fillColor={255,213,170},
                fillPattern=FillPattern.Solid,
                textString="%k"), Text(
                extent={{-72,68},{92,-68}},
                lineColor={0,0,0},
                fillColor={255,213,170},
                fillPattern=FillPattern.Solid,
                textString="+")}), Documentation(info="<html>
<p>
This blocks computes the scalar Real output \"y\" as sum of the elements of the
Real input signal vector u:
</p>
<blockquote><pre>
y = k[1]*u[1] + k[2]*u[2] + ... k[N]*u[N];
</pre></blockquote>

<p>
The input connector is a vector of Real input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.RealNetwork1\">Modelica.Blocks.Examples.RealNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to zero: y=0.
</p>

</html>"));
      end MultiSum;

      block Add "Output the sum of the two inputs"
        extends Interfaces.SI2SO;

        parameter Real k1=+1 "Gain of upper input";
        parameter Real k2=+1 "Gain of lower input";

      equation
        y = k1*u1 + k2*u2;
        annotation (
          Documentation(info="<html>
<p>
This blocks computes output <b>y</b> as <i>sum</i> of the
two input signals <b>u1</b> and <b>u2</b>:
</p>
<pre>
    <b>y</b> = k1*<b>u1</b> + k2*<b>u2</b>;
</pre>
<p>
Example:
</p>
<pre>
     parameter:   k1= +2, k2= -3

  results in the following equations:

     y = 2 * u1 - 3 * u2
</pre>

</html>"),Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Text(
                lineColor={0,0,255},
                extent={{-150,110},{150,150}},
                textString="%name"),
              Line(points={{-100,60},{-74,24},{-44,24}}, color={0,0,127}),
              Line(points={{-100,-60},{-74,-28},{-42,-28}}, color={0,0,127}),
              Ellipse(lineColor={0,0,127}, extent={{-50,-50},{50,50}}),
              Line(points={{50,0},{100,0}}, color={0,0,127}),
              Text(extent={{-38,-34},{38,34}}, textString="+"),
              Text(extent={{-100,52},{5,92}}, textString="%k1"),
              Text(extent={{-100,-92},{5,-52}}, textString="%k2")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                  extent={{-100,-100},{100,100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(points={{50,0},{100,0}},
                color={0,0,255}),Line(points={{-100,60},{-74,24},{-44,24}}, color={
                0,0,127}),Line(points={{-100,-60},{-74,-28},{-42,-28}}, color={0,0,
                127}),Ellipse(extent={{-50,50},{50,-50}}, lineColor={0,0,127}),Line(
                points={{50,0},{100,0}}, color={0,0,127}),Text(
                  extent={{-36,38},{40,-30}},
                  lineColor={0,0,0},
                  textString="+"),Text(
                  extent={{-100,52},{5,92}},
                  lineColor={0,0,0},
                  textString="k1"),Text(
                  extent={{-100,-52},{5,-92}},
                  lineColor={0,0,0},
                  textString="k2")}));
      end Add;
      annotation (Documentation(info="<html>
<p>
This package contains basic <b>mathematical operations</b>,
such as summation and multiplication, and basic <b>mathematical
functions</b>, such as <b>sqrt</b> and <b>sin</b>, as
input/output blocks. All blocks of this library can be either
connected with continuous blocks or with sampled-data blocks.
</p>
</html>",     revisions="<html>
<ul>
<li><i>October 21, 2002</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       New blocks added: RealToInteger, IntegerToReal, Max, Min, Edge, BooleanChange, IntegerChange.</li>
<li><i>August 7, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized (partly based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist).
</li>
</ul>
</html>"),     Icon(graphics={Line(
              points={{-80,-2},{-68.7,32.2},{-61.5,51.1},{-55.1,64.4},{-49.4,72.6},
                  {-43.8,77.1},{-38.2,77.8},{-32.6,74.6},{-26.9,67.7},{-21.3,57.4},
                  {-14.9,42.1},{-6.83,19.2},{10.1,-32.8},{17.3,-52.2},{23.7,-66.2},
                  {29.3,-75.1},{35,-80.4},{40.6,-82},{46.2,-79.6},{51.9,-73.5},{
                  57.5,-63.9},{63.9,-49.2},{72,-26.8},{80,-2}},
              color={95,95,95},
              smooth=Smooth.Bezier)}));
    end Math;

    package Nonlinear
    "Library of discontinuous or non-differentiable algebraic control blocks"
      import Modelica.Blocks.Interfaces;
      extends Modelica.Icons.Package;

          block Limiter "Limit the range of a signal"
            parameter Real uMax(start=1) "Upper limits of input signals";
            parameter Real uMin= -uMax "Lower limits of input signals";
            parameter Boolean strict=false
          "= true, if strict limits with noEvent(..)"
              annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
            parameter Boolean limitsAtInit=true
          "Has no longer an effect and is only kept for backwards compatibility (the implementation uses now the homotopy operator)"
              annotation (Dialog(tab="Dummy"),Evaluate=true, choices(checkBox=true));
            extends Interfaces.SISO;

          equation
            assert(uMax >= uMin, "Limiter: Limits must be consistent. However, uMax (=" + String(uMax) +
                                 ") < uMin (=" + String(uMin) + ")");
            if strict then
               y = homotopy(actual = smooth(0, noEvent(if u > uMax then uMax else if u < uMin then uMin else u)), simplified=u);
            else
               y = homotopy(actual = smooth(0,if u > uMax then uMax else if u < uMin then uMin else u), simplified=u);
            end if;
            annotation (
              Documentation(info="<html>
<p>
The Limiter block passes its input signal as output signal
as long as the input is within the specified upper and lower
limits. If this is not the case, the corresponding limits are passed
as output.
</p>
</html>"),           Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{0,-90},{0,68}}, color={192,192,192}),
              Polygon(
                points={{0,90},{-8,68},{8,68},{0,90}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-90,0},{68,0}}, color={192,192,192}),
              Polygon(
                points={{90,0},{68,-8},{68,8},{90,0}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,-70},{-50,-70},{50,70},{80,70}}),
              Text(
                extent={{-150,-150},{150,-110}},
                lineColor={0,0,0},
                textString="uMax=%uMax"),
              Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255}),
              Line(
                visible=strict,
                points={{50,70},{80,70}},
                color={255,0,0}),
              Line(
                visible=strict,
                points={{-80,-70},{-50,-70}},
                color={255,0,0})}),
              Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{0,-60},{0,50}}, color={192,192,192}),
              Polygon(
                points={{0,60},{-5,50},{5,50},{0,60}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-60,0},{50,0}}, color={192,192,192}),
              Polygon(
                points={{60,0},{50,-5},{50,5},{60,0}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-50,-40},{-30,-40},{30,40},{50,40}}),
              Text(
                extent={{46,-6},{68,-18}},
                lineColor={128,128,128},
                textString="u"),
              Text(
                extent={{-30,70},{-5,50}},
                lineColor={128,128,128},
                textString="y"),
              Text(
                extent={{-58,-54},{-28,-42}},
                lineColor={128,128,128},
                textString="uMin"),
              Text(
                extent={{26,40},{66,56}},
                lineColor={128,128,128},
                textString="uMax")}));
          end Limiter;
          annotation (
            Documentation(info="<html>
<p>
This package contains <b>discontinuous</b> and
<b>non-differentiable, algebraic</b> input/output blocks.
</p>
</html>",     revisions="<html>
<ul>
<li><i>October 21, 2002</i>
       by Christian Schweiger:<br>
       New block VariableLimiter added.</li>
<li><i>August 22, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.
</li>
</ul>
</html>"),     Icon(graphics={Line(points={{-80,-66},{-26,-66},{28,52},{88,52}},
                color={95,95,95})}));
    end Nonlinear;

    package Routing "Library of blocks to combine and extract signals"
      extends Modelica.Icons.Package;

      model BooleanPassThrough "Pass a Boolean signal through without modification"
        extends Modelica.Blocks.Interfaces.BooleanSISO;
      equation
        y = u;
        annotation (                         Documentation(info="<html>
<p>Passes a Boolean signal through without modification.  Enables signals to be read out of one bus, have their name changed and be sent back to a bus.</p>
</html>"),Icon(
            coordinateSystem(preserveAspectRatio=true,
                extent={{-100.0,-100.0},{100.0,100.0}}),
                graphics={
            Line(
              points={{-100.0,0.0},{100.0,0.0}},
              color={255,0,255})}));
      end BooleanPassThrough;
      annotation (Documentation(info="<html>
<p>
This package contains blocks to combine and extract signals.
</p>
</html>"),     Icon(graphics={
            Line(points={{-90,0},{4,0}}, color={95,95,95}),
            Line(points={{88,65},{48,65},{-8,0}}, color={95,95,95}),
            Line(points={{-8,0},{93,0}}, color={95,95,95}),
            Line(points={{87,-65},{48,-65},{-8,0}}, color={95,95,95})}));
    end Routing;

    package Sources
    "Library of signal source blocks generating Real and Boolean signals"
      import Modelica.Blocks.Interfaces;
      import Modelica.SIunits;
      extends Modelica.Icons.SourcesPackage;

      block RealExpression "Set output signal to a time varying Real expression"

        Modelica.Blocks.Interfaces.RealOutput y=0.0 "Value of Real output"
          annotation (Dialog(group="Time varying output signal"), Placement(
              transformation(extent={{100,-10},{120,10}})));

        annotation (Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,40},{100,-40}},
                lineColor={0,0,0},
                lineThickness=5.0,
                fillColor={235,235,235},
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised),
              Text(
                extent={{-96,15},{96,-15}},
                lineColor={0,0,0},
                textString="%y"),
              Text(
                extent={{-150,90},{140,50}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(info="<html>
<p>
The (time varying) Real output signal of this block can be defined in its
parameter menu via variable <b>y</b>. The purpose is to support the
easy definition of Real expressions in a block diagram. For example,
in the y-menu the definition \"if time &lt; 1 then 0 else 1\" can be given in order
to define that the output signal is one, if time &ge; 1 and otherwise
it is zero. Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
Variable <b>y</b> is both a variable and a connector.
</p>
</html>"));

      end RealExpression;

      block BooleanExpression
        "Set output signal to a time varying Boolean expression"

        Modelica.Blocks.Interfaces.BooleanOutput y=false "Value of Boolean output"
          annotation (Dialog(group="Time varying output signal"), Placement(
              transformation(extent={{100,-10},{120,10}})));

        annotation (Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,40},{100,-40}},
                lineColor={0,0,0},
                lineThickness=5.0,
                fillColor={235,235,235},
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised),
              Text(
                extent={{-96,15},{96,-15}},
                lineColor={0,0,0},
                textString="%y"),
              Text(
                extent={{-150,90},{140,50}},
                textString="%name",
                lineColor={0,0,255}),
              Polygon(
                points={{100,10},{120,0},{100,-10},{100,10}},
                lineColor=DynamicSelect({255,0,255}, if y > 0.5 then {0,255,0}
                     else {255,0,255}),
                fillColor=DynamicSelect({255,255,255}, if y > 0.5 then {0,255,0}
                     else {255,255,255}),
                fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
The (time varying) Boolean output signal of this block can be defined in its
parameter menu via variable <b>y</b>. The purpose is to support the
easy definition of Boolean expressions in a block diagram. For example,
in the y-menu the definition \"time &gt;= 1 and time &lt;= 2\" can be given in order
to define that the output signal is <b>true</b> in the time interval
1 &le; time &le; 2 and otherwise it is <b>false</b>.
Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
Variable <b>y</b> is both a variable and a connector.
</p>
</html>"));

      end BooleanExpression;

      block Constant "Generate constant signal of type Real"
        parameter Real k(start=1) "Constant output value";
        extends Interfaces.SO;

      equation
        y = k;
        annotation (
          defaultComponentName="const",
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
              Polygon(
                points={{-80,90},{-88,68},{-72,68},{-80,90}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
              Polygon(
                points={{90,-70},{68,-62},{68,-78},{90,-70}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,0},{80,0}}),
              Text(
                extent={{-150,-150},{150,-110}},
                lineColor={0,0,0},
                textString="k=%k")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Polygon(
                points={{-80,90},{-86,68},{-74,68},{-80,90}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,68},{-80,-80}}, color={95,95,95}),
              Line(
                points={{-80,0},{80,0}},
                color={0,0,255},
                thickness=0.5),
              Line(points={{-90,-70},{82,-70}}, color={95,95,95}),
              Polygon(
                points={{90,-70},{68,-64},{68,-76},{90,-70}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-83,92},{-30,74}},
                lineColor={0,0,0},
                textString="y"),
              Text(
                extent={{70,-80},{94,-100}},
                lineColor={0,0,0},
                textString="time"),
              Text(
                extent={{-101,8},{-81,-12}},
                lineColor={0,0,0},
                textString="k")}),
          Documentation(info="<html>
<p>
The Real output y is a constant signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Constant.png\"
     alt=\"Constant.png\">
</p>
</html>"));
      end Constant;

      block Step "Generate step signal of type Real"
        parameter Real height=1 "Height of step";
        extends Interfaces.SignalSource;

      equation
        y = offset + (if time < startTime then 0 else height);
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
              Polygon(
                points={{-80,90},{-88,68},{-72,68},{-80,90}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
              Polygon(
                points={{90,-70},{68,-62},{68,-78},{90,-70}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,-70},{0,-70},{0,50},{80,50}}),
              Text(
                extent={{-150,-150},{150,-110}},
                lineColor={0,0,0},
                textString="startTime=%startTime")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Polygon(
                points={{-80,90},{-86,68},{-74,68},{-80,90}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,68},{-80,-80}}, color={95,95,95}),
              Line(
                points={{-80,-18},{0,-18},{0,50},{80,50}},
                color={0,0,255},
                thickness=0.5),
              Line(points={{-90,-70},{82,-70}}, color={95,95,95}),
              Polygon(
                points={{90,-70},{68,-64},{68,-76},{90,-70}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{70,-80},{94,-100}},
                lineColor={0,0,0},
                textString="time"),
              Text(
                extent={{-21,-72},{25,-90}},
                lineColor={0,0,0},
                textString="startTime"),
              Line(points={{0,-18},{0,-70}}, color={95,95,95}),
              Text(
                extent={{-68,-36},{-22,-54}},
                lineColor={0,0,0},
                textString="offset"),
              Line(points={{-13,50},{-13,-17}}, color={95,95,95}),
              Polygon(
                points={{0,50},{-21,50},{0,50}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-13,-18},{-16,-5},{-10,-5},{-13,-18},{-13,-18}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-13,50},{-16,37},{-10,37},{-13,50}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-68,26},{-22,8}},
                lineColor={0,0,0},
                textString="height"),
              Polygon(
                points={{-13,-70},{-16,-57},{-10,-57},{-13,-70},{-13,-70}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Line(points={{-13,-18},{-13,-70}}, color={95,95,95}),
              Polygon(
                points={{-13,-18},{-16,-31},{-10,-31},{-13,-18}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-72,100},{-31,80}},
                lineColor={0,0,0},
                textString="y")}),
          Documentation(info="<html>
<p>
The Real output y is a step signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Step.png\"
     alt=\"Step.png\">
</p>

</html>"));
      end Step;

      block BooleanStep "Generate step signal of type Boolean"
        parameter Modelica.SIunits.Time startTime=0 "Time instant of step start";
        parameter Boolean startValue=false "Output before startTime";

        extends Interfaces.partialBooleanSource;
      equation
        y = if time >= startTime then not startValue else startValue;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}), graphics={
              Line(
                visible=not startValue,
                points={{-80,-70},{0,-70},{0,50},{80,50}}),
              Line(
                visible=startValue,
                points={{-80,50},{0,50},{0,-70},{68,-70}}),
              Text(
                extent={{-150,-140},{150,-110}},
                lineColor={0,0,0},
                textString="%startTime")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}), graphics={Line(
                  points={{-80,-70},{0,-70},{0,50},{80,50}},
                  color={0,0,255},
                  thickness=0.5),Text(
                  extent={{-15,-74},{20,-82}},
                  lineColor={0,0,0},
                  textString="startTime"),Polygon(
                  points={{2,50},{-80,50},{2,50}},
                  lineColor={95,95,95},
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-66,62},{-22,48}},
                  lineColor={0,0,0},
                  textString="not startValue"),Text(
                  extent={{-68,-58},{-36,-72}},
                  lineColor={0,0,0},
                  textString="startValue")}),
          Documentation(info="<html>
<p>
The Boolean output y is a step signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/BooleanStep.png\"
     alt=\"BooleanStep.png\">
</p>
</html>"));
      end BooleanStep;
      annotation (Documentation(info="<html>
<p>
This package contains <b>source</b> components, i.e., blocks which
have only output signals. These blocks are used as signal generators
for Real, Integer and Boolean signals.
</p>

<p>
All Real source signals (with the exception of the Constant source)
have at least the following two parameters:
</p>

<table border=1 cellspacing=0 cellpadding=2>
  <tr><td valign=\"top\"><b>offset</b></td>
      <td valign=\"top\">Value which is added to the signal</td>
  </tr>
  <tr><td valign=\"top\"><b>startTime</b></td>
      <td valign=\"top\">Start time of signal. For time &lt; startTime,
                the output y is set to offset.</td>
  </tr>
</table>

<p>
The <b>offset</b> parameter is especially useful in order to shift
the corresponding source, such that at initial time the system
is stationary. To determine the corresponding value of offset,
usually requires a trimming calculation.
</p>
</html>",     revisions="<html>
<ul>
<li><i>October 21, 2002</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Integer sources added. Step, TimeTable and BooleanStep slightly changed.</li>
<li><i>Nov. 8, 1999</i>
       by <a href=\"mailto:clauss@eas.iis.fhg.de\">Christoph Clau&szlig;</a>,
       <a href=\"mailto:Andre.Schneider@eas.iis.fraunhofer.de\">Andre.Schneider@eas.iis.fraunhofer.de</a>,
       <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       New sources: Exponentials, TimeTable. Trapezoid slightly enhanced
       (nperiod=-1 is an infinite number of periods).</li>
<li><i>Oct. 31, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       <a href=\"mailto:clauss@eas.iis.fhg.de\">Christoph Clau&szlig;</a>,
       <a href=\"mailto:Andre.Schneider@eas.iis.fraunhofer.de\">Andre.Schneider@eas.iis.fraunhofer.de</a>,
       All sources vectorized. New sources: ExpSine, Trapezoid,
       BooleanConstant, BooleanStep, BooleanPulse, SampleTrigger.
       Improved documentation, especially detailed description of
       signals in diagram layer.</li>
<li><i>June 29, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
    end Sources;

    package Types
    "Library of constants and types with choices, especially to build menus"
      extends Modelica.Icons.TypesPackage;

      type Init = enumeration(
          NoInit
            "No initialization (start values are used as guess values with fixed=false)",
          SteadyState
            "Steady state initialization (derivatives of states are zero)",
          InitialState "Initialization with initial states",
          InitialOutput
            "Initialization with initial outputs (and steady state of the states if possible)")
        "Enumeration defining initialization of a block" annotation (Evaluate=true,
        Documentation(info="<html>
  <p>The following initialization alternatives are available:</p>
  <dl>
    <dt><code><strong>NoInit</strong></code></dt>
      <dd>No initialization (start values are used as guess values with <code>fixed=false</code>)</dd>
    <dt><code><strong>SteadyState</strong></code></dt>
      <dd>Steady state initialization (derivatives of states are zero)</dd>
    <dt><code><strong>InitialState</strong></code></dt>
      <dd>Initialization with initial states</dd>
    <dt><code><strong>InitialOutput</strong></code></dt>
      <dd>Initialization with initial outputs (and steady state of the states if possible)</dd>
  </dl>
</html>"));
      annotation (Documentation(info="<html>
<p>
In this package <b>types</b>, <b>constants</b> and <b>external objects</b> are defined that are used
in library Modelica.Blocks. The types have additional annotation choices
definitions that define the menus to be built up in the graphical
user interface when the type is used as parameter in a declaration.
</p>
</html>"));
    end Types;

    package Icons "Icons for Blocks"
        extends Modelica.Icons.IconsPackage;

        partial block Block "Basic graphical layout of input/output block"

          annotation (
            Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                extent={{-100,-100},{100,100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255})}),
          Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output
block (no declarations, no equations). Most blocks
of package Modelica.Blocks inherit directly or indirectly
from this block.
</p>
</html>"));

        end Block;

        partial block BooleanBlock "Basic graphical layout of Boolean block"

          annotation (
            Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                extent={{-100,-100},{100,100}},
                lineColor={255,0,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255})}),
          Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output,
Boolean block (no declarations, no equations).
</p>
</html>"));

        end BooleanBlock;

      partial block PartialBooleanBlock "Basic graphical layout of logical block"

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Rectangle(
                extent={{-100,100},{100,-100}},
                fillColor={210,210,210},
                lineThickness=5.0,
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised), Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255})}),                        Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output,
Boolean block (no declarations, no equations) used especially
in the Blocks.Logical library.
</p>
</html>"));
      end PartialBooleanBlock;
    end Icons;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
        Rectangle(
          origin={0.0,35.1488},
          fillColor={255,255,255},
          extent={{-30.0,-20.1488},{30.0,20.1488}}),
        Rectangle(
          origin={0.0,-34.8512},
          fillColor={255,255,255},
          extent={{-30.0,-20.1488},{30.0,20.1488}}),
        Line(
          origin={-51.25,0.0},
          points={{21.25,-35.0},{-13.75,-35.0},{-13.75,35.0},{6.25,35.0}}),
        Polygon(
          origin={-40.0,35.0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{10.0,0.0},{-5.0,5.0},{-5.0,-5.0}}),
        Line(
          origin={51.25,0.0},
          points={{-21.25,35.0},{13.75,35.0},{13.75,-35.0},{-6.25,-35.0}}),
        Polygon(
          origin={40.0,-35.0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-10.0,0.0},{5.0,5.0},{5.0,-5.0}})}), Documentation(info="<html>
<p>
This library contains input/output blocks to build up block diagrams.
</p>

<dl>
<dt><b>Main Author:</b></dt>
<dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a><br>
    Deutsches Zentrum f&uuml;r Luft und Raumfahrt e. V. (DLR)<br>
    Oberpfaffenhofen<br>
    Postfach 1116<br>
    D-82230 Wessling<br>
    email: <A HREF=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</A><br></dd>
</dl>
<p>
Copyright &copy; 1998-2016, Modelica Association and DLR.
</p>
<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>
</html>",   revisions="<html>
<ul>
<li><i>June 23, 2004</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Introduced new block connectors and adapted all blocks to the new connectors.
       Included subpackages Continuous, Discrete, Logical, Nonlinear from
       package ModelicaAdditions.Blocks.
       Included subpackage ModelicaAdditions.Table in Modelica.Blocks.Sources
       and in the new package Modelica.Blocks.Tables.
       Added new blocks to Blocks.Sources and Blocks.Logical.
       </li>
<li><i>October 21, 2002</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       New subpackage Examples, additional components.
       </li>
<li><i>June 20, 2000</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and
       Michael Tiller:<br>
       Introduced a replaceable signal type into
       Blocks.Interfaces.RealInput/RealOutput:
<pre>
   replaceable type SignalType = Real
</pre>
       in order that the type of the signal of an input/output block
       can be changed to a physical type, for example:
<pre>
   Sine sin1(outPort(redeclare type SignalType=Modelica.SIunits.Torque))
</pre>
      </li>
<li><i>Sept. 18, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Renamed to Blocks. New subpackages Math, Nonlinear.
       Additional components in subpackages Interfaces, Continuous
       and Sources. </li>
<li><i>June 30, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
  end Blocks;

  package Math
  "Library of mathematical functions (e.g., sin, cos) and of functions operating on vectors and matrices"
  import SI = Modelica.SIunits;
  extends Modelica.Icons.Package;

  package Icons "Icons for Math"
    extends Modelica.Icons.IconsPackage;

    partial function AxisLeft
      "Basic icon for mathematical function with y-axis on left side"

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-80},{-80,68}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              lineColor={0,0,255})}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Line(points={{-80,80},{-88,80}}, color={95,
              95,95}),Line(points={{-80,-80},{-88,-80}}, color={95,95,95}),Line(
              points={{-80,-90},{-80,84}}, color={95,95,95}),Text(
                  extent={{-75,104},{-55,84}},
                  lineColor={95,95,95},
                  textString="y"),Polygon(
                  points={{-80,98},{-86,82},{-74,82},{-80,98}},
                  lineColor={95,95,95},
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
Icon for a mathematical function, consisting of an y-axis on the left side.
It is expected, that an x-axis is added and a plot of the function.
</p>
</html>"));
    end AxisLeft;

    partial function AxisCenter
      "Basic icon for mathematical function with y-axis in the center"

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              lineColor={0,0,255})}),
        Diagram(graphics={Line(points={{0,80},{-8,80}}, color={95,95,95}),Line(
              points={{0,-80},{-8,-80}}, color={95,95,95}),Line(points={{0,-90},{
              0,84}}, color={95,95,95}),Text(
                  extent={{5,104},{25,84}},
                  lineColor={95,95,95},
                  textString="y"),Polygon(
                  points={{0,98},{-6,82},{6,82},{0,98}},
                  lineColor={95,95,95},
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
Icon for a mathematical function, consisting of an y-axis in the middle.
It is expected, that an x-axis is added and a plot of the function.
</p>
</html>"));
    end AxisCenter;
  end Icons;

  function asin "Inverse sine (-1 <= u <= 1)"
    extends Modelica.Math.Icons.AxisCenter;
    input Real u;
    output SI.Angle y;

  external "builtin" y = asin(u);
    annotation (
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-90,0},{68,0}}, color={192,192,192}),
          Polygon(
            points={{90,0},{68,8},{68,-8},{90,0}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,-80},{-79.2,-72.8},{-77.6,-67.5},{-73.6,-59.4},{-66.3,
                -49.8},{-53.5,-37.3},{-30.2,-19.7},{37.4,24.8},{57.5,40.8},{68.7,
                52.7},{75.2,62.2},{77.6,67.5},{80,80}}),
          Text(
            extent={{-88,78},{-16,30}},
            lineColor={192,192,192},
            textString="asin")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Text(
              extent={{-40,-72},{-15,-88}},
              textString="-pi/2",
              lineColor={0,0,255}),Text(
              extent={{-38,88},{-13,72}},
              textString=" pi/2",
              lineColor={0,0,255}),Text(
              extent={{68,-9},{88,-29}},
              textString="+1",
              lineColor={0,0,255}),Text(
              extent={{-90,21},{-70,1}},
              textString="-1",
              lineColor={0,0,255}),Line(points={{-100,0},{84,0}}, color={95,95,95}),
            Polygon(
              points={{98,0},{82,6},{82,-6},{98,0}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-80,-80},{-79.2,-72.8},{-77.6,-67.5},{-73.6,-59.4},{-66.3,
              -49.8},{-53.5,-37.3},{-30.2,-19.7},{37.4,24.8},{57.5,40.8},{68.7,
              52.7},{75.2,62.2},{77.6,67.5},{80,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{82,24},{102,4}},
              lineColor={95,95,95},
              textString="u"),Line(
              points={{0,80},{86,80}},
              color={175,175,175}),Line(
              points={{80,86},{80,-10}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = asin(u), with -1 &le; u &le; +1:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/asin.png\">
</p>
</html>"));
  end asin;

  function tanh "Hyperbolic tangent"
    extends Modelica.Math.Icons.AxisCenter;
    input Real u;
    output Real y;

  external "builtin" y = tanh(u);
    annotation (
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-90,0},{68,0}}, color={192,192,192}),
          Polygon(
            points={{90,0},{68,8},{68,-8},{90,0}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,-80},{-47.8,-78.7},{-35.8,-75.7},{-27.7,-70.6},{-22.1,
                -64.2},{-17.3,-55.9},{-12.5,-44.3},{-7.64,-29.2},{-1.21,-4.82},{
                6.83,26.3},{11.7,42},{16.5,54.2},{21.3,63.1},{26.9,69.9},{34.2,75},
                {45.4,78.4},{72,79.9},{80,80}}),
          Text(
            extent={{-88,72},{-16,24}},
            lineColor={192,192,192},
            textString="tanh")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Line(points={{-100,0},{84,0}}, color={95,95,
            95}),Polygon(
              points={{96,0},{80,6},{80,-6},{96,0}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-80,-80.5},{-47.8,-79.2},{-35.8,-76.2},{-27.7,-71.1},{-22.1,
              -64.7},{-17.3,-56.4},{-12.5,-44.8},{-7.64,-29.7},{-1.21,-5.32},{
              6.83,25.8},{11.7,41.5},{16.5,53.7},{21.3,62.6},{26.9,69.4},{34.2,
              74.5},{45.4,77.9},{72,79.4},{80,79.5}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{-29,72},{-9,88}},
              textString="1",
              lineColor={0,0,255}),Text(
              extent={{3,-72},{23,-88}},
              textString="-1",
              lineColor={0,0,255}),Text(
              extent={{82,-2},{102,-22}},
              lineColor={95,95,95},
              textString="u"),Line(
              points={{0,80},{88,80}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = tanh(u), with -&infin; &lt; u &lt; &infin;:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/tanh.png\">
</p>
</html>"));
  end tanh;

  function log "Natural (base e) logarithm (u shall be > 0)"
    extends Modelica.Math.Icons.AxisLeft;
    input Real u;
    output Real y;

  external "builtin" y = log(u);
    annotation (
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-90,0},{68,0}}, color={192,192,192}),
          Polygon(
            points={{90,0},{68,8},{68,-8},{90,0}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,-80},{-79.2,-50.6},{-78.4,-37},{-77.6,-28},{-76.8,-21.3},
                {-75.2,-11.4},{-72.8,-1.31},{-69.5,8.08},{-64.7,17.9},{-57.5,28},
                {-47,38.1},{-31.8,48.1},{-10.1,58},{22.1,68},{68.7,78.1},{80,80}}),
          Text(
            extent={{-6,-24},{66,-72}},
            lineColor={192,192,192},
            textString="log")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Line(points={{-100,0},{84,0}}, color={95,95,95}),
            Polygon(
              points={{100,0},{84,6},{84,-6},{100,0}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-78,-80},{-77.2,-50.6},{-76.4,-37},{-75.6,-28},{-74.8,-21.3},
              {-73.2,-11.4},{-70.8,-1.31},{-67.5,8.08},{-62.7,17.9},{-55.5,28},{-45,
              38.1},{-29.8,48.1},{-8.1,58},{24.1,68},{70.7,78.1},{82,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{-105,72},{-85,88}},
              textString="3",
              lineColor={0,0,255}),Text(
              extent={{60,-3},{80,-23}},
              textString="20",
              lineColor={0,0,255}),Text(
              extent={{-78,-7},{-58,-27}},
              textString="1",
              lineColor={0,0,255}),Text(
              extent={{84,26},{104,6}},
              lineColor={95,95,95},
              textString="u"),Text(
              extent={{-100,9},{-80,-11}},
              textString="0",
              lineColor={0,0,255}),Line(
              points={{-80,80},{84,80}},
              color={175,175,175}),Line(
              points={{82,82},{82,-6}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = log(10) (the natural logarithm of u),
with u &gt; 0:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/log.png\">
</p>
</html>"));
  end log;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={Line(points={{-80,0},{-68.7,34.2},{-61.5,53.1},
              {-55.1,66.4},{-49.4,74.6},{-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{
              -26.9,69.7},{-21.3,59.4},{-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,
              -50.2},{23.7,-64.2},{29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},
              {51.9,-71.5},{57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}}, color={
              0,0,0}, smooth=Smooth.Bezier)}), Documentation(info="<html>
<p>
This package contains <b>basic mathematical functions</b> (such as sin(..)),
as well as functions operating on
<a href=\"modelica://Modelica.Math.Vectors\">vectors</a>,
<a href=\"modelica://Modelica.Math.Matrices\">matrices</a>,
<a href=\"modelica://Modelica.Math.Nonlinear\">nonlinear functions</a>, and
<a href=\"modelica://Modelica.Math.BooleanVectors\">Boolean vectors</a>.
</p>

<dl>
<dt><b>Main Authors:</b></dt>
<dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and
    Marcus Baur<br>
    Deutsches Zentrum f&uuml;r Luft und Raumfahrt e.V. (DLR)<br>
    Institut f&uuml;r Robotik und Mechatronik<br>
    Postfach 1116<br>
    D-82230 Wessling<br>
    Germany<br>
    email: <A HREF=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</A><br></dd>
</dl>

<p>
Copyright &copy; 1998-2016, Modelica Association and DLR.
</p>
<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>
</html>",   revisions="<html>
<ul>
<li><i>October 21, 2002</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Function tempInterpol2 added.</li>
<li><i>Oct. 24, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Icons for icon and diagram level introduced.</li>
<li><i>June 30, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized.</li>
</ul>

</html>"));
  end Math;

  package Constants
  "Library of mathematical constants and constants of nature (e.g., pi, eps, R, sigma)"
    import SI = Modelica.SIunits;
    import NonSI = Modelica.SIunits.Conversions.NonSIunits;
    extends Modelica.Icons.Package;

    final constant Real pi=2*Modelica.Math.asin(1.0);

    final constant Real eps=ModelicaServices.Machine.eps
      "Biggest number such that 1.0 + eps = 1.0";

    final constant Real small=ModelicaServices.Machine.small
      "Smallest number such that small and -small are representable on the machine";

    final constant SI.Acceleration g_n=9.80665
      "Standard acceleration of gravity on earth";

    final constant NonSI.Temperature_degC T_zero=-273.15
      "Absolute zero temperature";
    annotation (
      Documentation(info="<html>
<p>
This package provides often needed constants from mathematics, machine
dependent constants and constants from nature. The latter constants
(name, value, description) are from the following source:
</p>

<dl>
<dt>Peter J. Mohr, David B. Newell, and Barry N. Taylor:</dt>
<dd><b>CODATA Recommended Values of the Fundamental Physical Constants: 2014</b>.
<a href= \"http://dx.doi.org/10.5281/zenodo.22826\">http://dx.doi.org/10.5281/zenodo.22826</a>, 2015. See also <a href=
\"http://physics.nist.gov/cuu/Constants/index.html\">http://physics.nist.gov/cuu/Constants/index.html</a></dd>
</dl>

<p>CODATA is the Committee on Data for Science and Technology.</p>

<dl>
<dt><b>Main Author:</b></dt>
<dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a><br>
    Deutsches Zentrum f&uuml;r Luft und Raumfahrt e. V. (DLR)<br>
    Oberpfaffenhofen<br>
    Postfach 1116<br>
    D-82230 We&szlig;ling<br>
    email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a></dd>
</dl>

<p>
Copyright &copy; 1998-2016, Modelica Association and DLR.
</p>
<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>
</html>",   revisions="<html>
<ul>
<li><i>Nov 4, 2015</i>
       by Thomas Beutlich:<br>
       Constants updated according to 2014 CODATA values.</li>
<li><i>Nov 8, 2004</i>
       by Christian Schweiger:<br>
       Constants updated according to 2002 CODATA values.</li>
<li><i>Dec 9, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Constants updated according to 1998 CODATA values. Using names, values
       and description text from this source. Included magnetic and
       electric constant.</li>
<li><i>Sep 18, 1999</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Constants eps, inf, small introduced.</li>
<li><i>Nov 15, 1997</i>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized.</li>
</ul>
</html>"),
      Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
        Polygon(
          origin={-9.2597,25.6673},
          fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{48.017,11.336},{48.017,11.336},{10.766,11.336},{-25.684,10.95},{-34.944,-15.111},{-34.944,-15.111},{-32.298,-15.244},{-32.298,-15.244},{-22.112,0.168},{11.292,0.234},{48.267,-0.097},{48.267,-0.097}},
          smooth=Smooth.Bezier),
        Polygon(
          origin={-19.9923,-8.3993},
          fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{3.239,37.343},{3.305,37.343},{-0.399,2.683},{-16.936,-20.071},{-7.808,-28.604},{6.811,-22.519},{9.986,37.145},{9.986,37.145}},
          smooth=Smooth.Bezier),
        Polygon(
          origin={23.753,-11.5422},
          fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-10.873,41.478},{-10.873,41.478},{-14.048,-4.162},{-9.352,-24.8},{7.912,-24.469},{16.247,0.27},{16.247,0.27},{13.336,0.071},{13.336,0.071},{7.515,-9.983},{-3.134,-7.271},{-2.671,41.214},{-2.671,41.214}},
          smooth=Smooth.Bezier)}));
  end Constants;

  package Icons "Library of icons"
    extends Icons.Package;

    partial package Package "Icon for standard packages"

      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100.0,-100.0},{100.0,100.0}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100.0,-100.0},{100.0,100.0}},
              radius=25.0)}),   Documentation(info="<html>
<p>Standard package icon.</p>
</html>"));
    end Package;

    partial package InterfacesPackage "Icon for packages containing interfaces"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Polygon(origin={20.0,0.0},
              lineColor={64,64,64},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              points={{-10.0,70.0},{10.0,70.0},{40.0,20.0},{80.0,20.0},{80.0,-20.0},{40.0,-20.0},{10.0,-70.0},{-10.0,-70.0}}),
            Polygon(fillColor={102,102,102},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-100.0,20.0},{-60.0,20.0},{-30.0,70.0},{-10.0,70.0},{-10.0,-70.0},{-30.0,-70.0},{-60.0,-20.0},{-100.0,-20.0}})}),
                                Documentation(info="<html>
<p>This icon indicates packages containing interfaces.</p>
</html>"));
    end InterfacesPackage;

    partial package SourcesPackage "Icon for packages containing sources"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Polygon(origin={23.3333,0.0},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-23.333,30.0},{46.667,0.0},{-23.333,-30.0}}),
            Rectangle(
              fillColor = {128,128,128},
              pattern = LinePattern.None,
              fillPattern = FillPattern.Solid,
              extent = {{-70,-4.5},{0,4.5}})}),
                                Documentation(info="<html>
<p>This icon indicates a package which contains sources.</p>
</html>"));
    end SourcesPackage;

    partial package TypesPackage "Icon for packages containing type definitions"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Polygon(
              origin={-12.167,-23},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{12.167,65},{14.167,93},{36.167,89},{24.167,20},{4.167,-30},
                  {14.167,-30},{24.167,-30},{24.167,-40},{-5.833,-50},{-15.833,
                  -30},{4.167,20},{12.167,65}},
              smooth=Smooth.Bezier,
              lineColor={0,0,0}), Polygon(
              origin={2.7403,1.6673},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{49.2597,22.3327},{31.2597,24.3327},{7.2597,18.3327},{-26.7403,
                10.3327},{-46.7403,14.3327},{-48.7403,6.3327},{-32.7403,0.3327},{-6.7403,
                4.3327},{33.2597,14.3327},{49.2597,14.3327},{49.2597,22.3327}},
              smooth=Smooth.Bezier)}));
    end TypesPackage;

    partial package IconsPackage "Icon for packages containing icons"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Polygon(
              origin={-8.167,-17},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-15.833,20.0},{-15.833,30.0},{14.167,40.0},{24.167,20.0},{
                  4.167,-30.0},{14.167,-30.0},{24.167,-30.0},{24.167,-40.0},{-5.833,
                  -50.0},{-15.833,-30.0},{4.167,20.0},{-5.833,20.0}},
              smooth=Smooth.Bezier,
              lineColor={0,0,0}), Ellipse(
              origin={-0.5,56.5},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-12.5,-12.5},{12.5,12.5}},
              lineColor={0,0,0})}));
    end IconsPackage;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Polygon(
              origin={-8.167,-17},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-15.833,20.0},{-15.833,30.0},{14.167,40.0},{24.167,20.0},{
                  4.167,-30.0},{14.167,-30.0},{24.167,-30.0},{24.167,-40.0},{-5.833,
                  -50.0},{-15.833,-30.0},{4.167,20.0},{-5.833,20.0}},
              smooth=Smooth.Bezier,
              lineColor={0,0,0}), Ellipse(
              origin={-0.5,56.5},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-12.5,-12.5},{12.5,12.5}},
              lineColor={0,0,0})}), Documentation(info="<html>
<p>This package contains definitions for the graphical layout of components which may be used in different libraries. The icons can be utilized by inheriting them in the desired class using &quot;extends&quot; or by directly copying the &quot;icon&quot; layer. </p>

<h4>Main Authors:</h4>

<dl>
<dt><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a></dt>
    <dd>Deutsches Zentrum fuer Luft und Raumfahrt e.V. (DLR)</dd>
    <dd>Oberpfaffenhofen</dd>
    <dd>Postfach 1116</dd>
    <dd>D-82230 Wessling</dd>
    <dd>email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a></dd>
<dt>Christian Kral</dt>

    <dd>  <a href=\"http://christiankral.net/\">Electric Machines, Drives and Systems</a><br>
</dd>
    <dd>1060 Vienna, Austria</dd>
    <dd>email: <a href=\"mailto:dr.christian.kral@gmail.com\">dr.christian.kral@gmail.com</a></dd>
<dt>Johan Andreasson</dt>
    <dd><a href=\"http://www.modelon.se/\">Modelon AB</a></dd>
    <dd>Ideon Science Park</dd>
    <dd>22370 Lund, Sweden</dd>
    <dd>email: <a href=\"mailto:johan.andreasson@modelon.se\">johan.andreasson@modelon.se</a></dd>
</dl>

<p>Copyright &copy; 1998-2016, Modelica Association, DLR, AIT, and Modelon AB. </p>
<p><i>This Modelica package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>, see the license conditions and the accompanying <b>disclaimer</b> in <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a>.</i> </p>
</html>"));
  end Icons;

  package SIunits
  "Library of type and unit definitions based on SI units according to ISO 31-1992"
    extends Modelica.Icons.Package;

    package Icons "Icons for SIunits"
      extends Modelica.Icons.IconsPackage;

      partial function Conversion "Base icon for conversion functions"

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={191,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-90,0},{30,0}}, color={191,0,0}),
              Polygon(
                points={{90,0},{30,20},{30,-20},{90,0}},
                lineColor={191,0,0},
                fillColor={191,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-115,155},{115,105}},
                textString="%name",
                lineColor={0,0,255})}));
      end Conversion;
    end Icons;

    package Conversions
    "Conversion functions to/from non SI units and type definitions of non SI units"
      extends Modelica.Icons.Package;

      package NonSIunits "Type definitions of non SI units"
        extends Modelica.Icons.Package;

        type Temperature_degC = Real (final quantity="ThermodynamicTemperature",
              final unit="degC")
          "Absolute temperature in degree Celsius (for relative temperature use SIunits.TemperatureDifference)"
                                                                                                              annotation(absoluteValue=true);

        type AngularVelocity_rpm = Real (final quantity="AngularVelocity", final unit=
                   "rev/min")
          "Angular velocity in revolutions per minute. Alias unit names that are outside of the SI system: rpm, r/min, rev/min";
        annotation (Documentation(info="<html>
<p>
This package provides predefined types, such as <b>Angle_deg</b> (angle in
degree), <b>AngularVelocity_rpm</b> (angular velocity in revolutions per
minute) or <b>Temperature_degF</b> (temperature in degree Fahrenheit),
which are in common use but are not part of the international standard on
units according to ISO 31-1992 \"General principles concerning quantities,
units and symbols\" and ISO 1000-1992 \"SI units and recommendations for
the use of their multiples and of certain other units\".</p>
<p>If possible, the types in this package should not be used. Use instead
types of package Modelica.SIunits. For more information on units, see also
the book of Francois Cardarelli <b>Scientific Unit Conversion - A
Practical Guide to Metrication</b> (Springer 1997).</p>
<p>Some units, such as <b>Temperature_degC/Temp_C</b> are both defined in
Modelica.SIunits and in Modelica.Conversions.NonSIunits. The reason is that these
definitions have been placed erroneously in Modelica.SIunits although they
are not SIunits. For backward compatibility, these type definitions are
still kept in Modelica.SIunits.</p>
</html>"),   Icon(coordinateSystem(extent={{-100,-100},{100,100}}), graphics={
        Text(
          origin={15.0,51.8518},
          extent={{-105.0,-86.8518},{75.0,-16.8518}},
          lineColor={0,0,0},
          textString="[km/h]")}));
      end NonSIunits;

      function to_degC "Convert from Kelvin to degCelsius"
        extends Modelica.SIunits.Icons.Conversion;
        input Temperature Kelvin "Kelvin value";
        output NonSIunits.Temperature_degC Celsius "Celsius value";
      algorithm
        Celsius := Kelvin + Modelica.Constants.T_zero;
        annotation (Inline=true,Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                extent={{-20,100},{-100,20}},
                lineColor={0,0,0},
                textString="K"), Text(
                extent={{100,-20},{20,-100}},
                lineColor={0,0,0},
                textString="degC")}));
      end to_degC;
      annotation (                              Documentation(info="<html>
<p>This package provides conversion functions from the non SI Units
defined in package Modelica.SIunits.Conversions.NonSIunits to the
corresponding SI Units defined in package Modelica.SIunits and vice
versa. It is recommended to use these functions in the following
way (note, that all functions have one Real input and one Real output
argument):</p>
<pre>
  <b>import</b> SI = Modelica.SIunits;
  <b>import</b> Modelica.SIunits.Conversions.*;
     ...
  <b>parameter</b> SI.Temperature     T   = from_degC(25);   // convert 25 degree Celsius to Kelvin
  <b>parameter</b> SI.Angle           phi = from_deg(180);   // convert 180 degree to radian
  <b>parameter</b> SI.AngularVelocity w   = from_rpm(3600);  // convert 3600 revolutions per minutes
                                                      // to radian per seconds
</pre>

</html>"));
    end Conversions;

    type Angle = Real (
        final quantity="Angle",
        final unit="rad",
        displayUnit="deg");

    type Length = Real (final quantity="Length", final unit="m");

    type Distance = Length (min=0);

    type Thickness = Length(min=0);

    type Diameter = Length(min=0);

    type Area = Real (final quantity="Area", final unit="m2");

    type Volume = Real (final quantity="Volume", final unit="m3");

    type Time = Real (final quantity="Time", final unit="s");

    type AngularVelocity = Real (
        final quantity="AngularVelocity",
        final unit="rad/s");

    type AngularAcceleration = Real (final quantity="AngularAcceleration", final unit=
               "rad/s2");

    type Velocity = Real (final quantity="Velocity", final unit="m/s");

    type Acceleration = Real (final quantity="Acceleration", final unit="m/s2");

    type Frequency = Real (final quantity="Frequency", final unit="Hz");

    type AngularFrequency = Real (final quantity="AngularFrequency", final unit=
            "rad/s");

    type DampingCoefficient = Real (final quantity="DampingCoefficient", final unit=
               "s-1");

    type Mass = Real (
        quantity="Mass",
        final unit="kg",
        min=0);

    type Density = Real (
        final quantity="Density",
        final unit="kg/m3",
        displayUnit="g/cm3",
        min=0.0);

    type MomentOfInertia = Real (final quantity="MomentOfInertia", final unit=
            "kg.m2");

    type Inertia = MomentOfInertia;

    type Torque = Real (final quantity="Torque", final unit="N.m");

    type Pressure = Real (
        final quantity="Pressure",
        final unit="Pa",
        displayUnit="bar");

    type AbsolutePressure = Pressure (min=0.0, nominal = 1e5);

    type DynamicViscosity = Real (
        final quantity="DynamicViscosity",
        final unit="Pa.s",
        min=0);

    type SurfaceTension = Real (final quantity="SurfaceTension", final unit="N/m");

    type Energy = Real (final quantity="Energy", final unit="J");

    type Power = Real (final quantity="Power", final unit="W");

    type Efficiency = Real (
        final quantity="Efficiency",
        final unit="1",
        min=0);

    type MassFlowRate = Real (quantity="MassFlowRate", final unit="kg/s");

    type VolumeFlowRate = Real (final quantity="VolumeFlowRate", final unit=
            "m3/s");

    type ThermodynamicTemperature = Real (
        final quantity="ThermodynamicTemperature",
        final unit="K",
        min = 0.0,
        start = 288.15,
        nominal = 300,
        displayUnit="degC")
      "Absolute temperature (use type TemperatureDifference for relative temperatures)"                   annotation(absoluteValue=true);

    type Temperature = ThermodynamicTemperature;

    type TemperatureDifference = Real (
        final quantity="ThermodynamicTemperature",
        final unit="K") annotation(absoluteValue=false);

    type Temp_C = SIunits.Conversions.NonSIunits.Temperature_degC;

    type LinearExpansionCoefficient = Real (final quantity=
            "LinearExpansionCoefficient", final unit="1/K");

    type Compressibility = Real (final quantity="Compressibility", final unit=
            "1/Pa");

    type Heat = Real (final quantity="Energy", final unit="J");

    type HeatFlowRate = Real (final quantity="Power", final unit="W");

    type ThermalConductivity = Real (final quantity="ThermalConductivity", final unit=
               "W/(m.K)");

    type CoefficientOfHeatTransfer = Real (final quantity=
            "CoefficientOfHeatTransfer", final unit="W/(m2.K)");

    type ThermalResistance = Real (final quantity="ThermalResistance", final unit=
           "K/W");

    type ThermalConductance = Real (final quantity="ThermalConductance", final unit=
               "W/K");

    type SpecificHeatCapacity = Real (final quantity="SpecificHeatCapacity",
          final unit="J/(kg.K)");

    type SpecificEntropy = Real (final quantity="SpecificEntropy",
                                 final unit="J/(kg.K)");

    type Enthalpy = Heat;

    type SpecificEnergy = Real (final quantity="SpecificEnergy",
                                final unit="J/kg");

    type SpecificEnthalpy = SpecificEnergy;

    type DerDensityByEnthalpy = Real (final unit="kg.s2/m5");

    type DerDensityByPressure = Real (final unit="s2/m2");

    type MolarMass = Real (final quantity="MolarMass", final unit="kg/mol",min=0);

    type MassFraction = Real (final quantity="MassFraction", final unit="1",
                              min=0, max=1);

    type MoleFraction = Real (final quantity="MoleFraction", final unit="1",
                              min = 0, max = 1);

    type PartialPressure = AbsolutePressure;

    type PrandtlNumber = Real (final quantity="PrandtlNumber", final unit="1");
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={
          Line(
            points={{-66,78},{-66,-40}},
            color={64,64,64}),
          Ellipse(
            extent={{12,36},{68,-38}},
            lineColor={64,64,64},
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-74,78},{-66,-40}},
            lineColor={64,64,64},
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-66,-4},{-66,6},{-16,56},{-16,46},{-66,-4}},
            lineColor={64,64,64},
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-46,16},{-40,22},{-2,-40},{-10,-40},{-46,16}},
            lineColor={64,64,64},
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{22,26},{58,-28}},
            lineColor={64,64,64},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{68,2},{68,-46},{64,-60},{58,-68},{48,-72},{18,-72},{18,-64},
                {46,-64},{54,-60},{58,-54},{60,-46},{60,-26},{64,-20},{68,-6},{68,
                2}},
            lineColor={64,64,64},
            smooth=Smooth.Bezier,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>This package provides predefined types, such as <i>Mass</i>,
<i>Angle</i>, <i>Time</i>, based on the international standard
on units, e.g.,
</p>

<pre>   <b>type</b> Angle = Real(<b>final</b> quantity = \"Angle\",
                     <b>final</b> unit     = \"rad\",
                     displayUnit    = \"deg\");
</pre>

<p>
Some of the types are derived SI units that are utilized in package Modelica
(such as ComplexCurrent, which is a complex number where both the real and imaginary
part have the SI unit Ampere).
</p>

<p>
Furthermore, conversion functions from non SI-units to SI-units and vice versa
are provided in subpackage
<a href=\"modelica://Modelica.SIunits.Conversions\">Conversions</a>.
</p>

<p>
For an introduction how units are used in the Modelica standard library
with package SIunits, have a look at:
<a href=\"modelica://Modelica.SIunits.UsersGuide.HowToUseSIunits\">How to use SIunits</a>.
</p>

<p>
Copyright &copy; 1998-2016, Modelica Association and DLR.
</p>
<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>
</html>",   revisions="<html>
<ul>
<li><i>May 25, 2011</i> by Stefan Wischhusen:<br/>Added molar units for energy and enthalpy.</li>
<li><i>Jan. 27, 2010</i> by Christian Kral:<br/>Added complex units.</li>
<li><i>Dec. 14, 2005</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Add User&#39;;s Guide and removed &quot;min&quot; values for Resistance and Conductance.</li>
<li><i>October 21, 2002</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and Christian Schweiger:<br/>Added new package <b>Conversions</b>. Corrected typo <i>Wavelenght</i>.</li>
<li><i>June 6, 2000</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Introduced the following new types<br/>type Temperature = ThermodynamicTemperature;<br/>types DerDensityByEnthalpy, DerDensityByPressure, DerDensityByTemperature, DerEnthalpyByPressure, DerEnergyByDensity, DerEnergyByPressure<br/>Attribute &quot;final&quot; removed from min and max values in order that these values can still be changed to narrow the allowed range of values.<br/>Quantity=&quot;Stress&quot; removed from type &quot;Stress&quot;, in order that a type &quot;Stress&quot; can be connected to a type &quot;Pressure&quot;.</li>
<li><i>Oct. 27, 1999</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>New types due to electrical library: Transconductance, InversePotential, Damping.</li>
<li><i>Sept. 18, 1999</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Renamed from SIunit to SIunits. Subpackages expanded, i.e., the SIunits package, does no longer contain subpackages.</li>
<li><i>Aug 12, 1999</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Type &quot;Pressure&quot; renamed to &quot;AbsolutePressure&quot; and introduced a new type &quot;Pressure&quot; which does not contain a minimum of zero in order to allow convenient handling of relative pressure. Redefined BulkModulus as an alias to AbsolutePressure instead of Stress, since needed in hydraulics.</li>
<li><i>June 29, 1999</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Bug-fix: Double definition of &quot;Compressibility&quot; removed and appropriate &quot;extends Heat&quot; clause introduced in package SolidStatePhysics to incorporate ThermodynamicTemperature.</li>
<li><i>April 8, 1998</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and Astrid Jaschinski:<br/>Complete ISO 31 chapters realized.</li>
<li><i>Nov. 15, 1997</i> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and Hubertus Tummescheit:<br/>Some chapters realized.</li>
</ul>
</html>"));
  end SIunits;
annotation (
preferredView="info",
version="3.2.2",
versionBuild=3,
versionDate="2016-04-03",
dateModified = "2016-04-03 08:44:41Z",
revisionId="$Id:: package.mo 9263 2016-04-03 18:10:55Z #$",
uses(Complex(version="3.2.2"), ModelicaServices(version="3.2.2")),
conversion(
 noneFromVersion="3.2.1",
 noneFromVersion="3.2",
 noneFromVersion="3.1",
 noneFromVersion="3.0.1",
 noneFromVersion="3.0",
 from(version="2.1", script="modelica://Modelica/Resources/Scripts/Dymola/ConvertModelica_from_2.2.2_to_3.0.mos"),
 from(version="2.2", script="modelica://Modelica/Resources/Scripts/Dymola/ConvertModelica_from_2.2.2_to_3.0.mos"),
 from(version="2.2.1", script="modelica://Modelica/Resources/Scripts/Dymola/ConvertModelica_from_2.2.2_to_3.0.mos"),
 from(version="2.2.2", script="modelica://Modelica/Resources/Scripts/Dymola/ConvertModelica_from_2.2.2_to_3.0.mos")),
Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
  Polygon(
    origin={-6.9888,20.048},
    fillColor={0,0,0},
    pattern=LinePattern.None,
    fillPattern=FillPattern.Solid,
    points={{-93.0112,10.3188},{-93.0112,10.3188},{-73.011,24.6},{-63.011,31.221},{-51.219,36.777},{-39.842,38.629},{-31.376,36.248},{-25.819,29.369},{-24.232,22.49},{-23.703,17.463},{-15.501,25.135},{-6.24,32.015},{3.02,36.777},{15.191,39.423},{27.097,37.306},{32.653,29.633},{35.035,20.108},{43.501,28.046},{54.085,35.19},{65.991,39.952},{77.897,39.688},{87.422,33.338},{91.126,21.696},{90.068,9.525},{86.099,-1.058},{79.749,-10.054},{71.283,-21.431},{62.816,-33.337},{60.964,-32.808},{70.489,-16.14},{77.368,-2.381},{81.072,10.054},{79.749,19.05},{72.605,24.342},{61.758,23.019},{49.587,14.817},{39.003,4.763},{29.214,-6.085},{21.012,-16.669},{13.339,-26.458},{5.401,-36.777},{-1.213,-46.037},{-6.24,-53.446},{-8.092,-52.387},{-0.684,-40.746},{5.401,-30.692},{12.81,-17.198},{19.424,-3.969},{23.658,7.938},{22.335,18.785},{16.514,23.283},{8.047,23.019},{-1.478,19.05},{-11.267,11.113},{-19.734,2.381},{-29.259,-8.202},{-38.519,-19.579},{-48.044,-31.221},{-56.511,-43.392},{-64.449,-55.298},{-72.386,-66.939},{-77.678,-74.612},{-79.53,-74.083},{-71.857,-61.383},{-62.861,-46.037},{-52.278,-28.046},{-44.869,-15.346},{-38.784,-2.117},{-35.344,8.731},{-36.403,19.844},{-42.488,23.813},{-52.013,22.49},{-60.744,16.933},{-68.947,10.054},{-76.884,2.646},{-93.0112,-12.1707},{-93.0112,-12.1707}},
    smooth=Smooth.Bezier),
  Ellipse(
    origin={40.8208,-37.7602},
    fillColor={161,0,4},
    pattern=LinePattern.None,
    fillPattern=FillPattern.Solid,
    extent={{-17.8562,-17.8563},{17.8563,17.8562}})}),
Documentation(info="<html>
<p>
Package <b>Modelica&reg;</b> is a <b>standardized</b> and <b>free</b> package
that is developed together with the Modelica&reg; language from the
Modelica Association, see
<a href=\"https://www.Modelica.org\">https://www.Modelica.org</a>.
It is also called <b>Modelica Standard Library</b>.
It provides model components in many domains that are based on
standardized interface definitions. Some typical examples are shown
in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/UsersGuide/ModelicaLibraries.png\">
</p>

<p>
For an introduction, have especially a look at:
</p>
<ul>
<li> <a href=\"modelica://Modelica.UsersGuide.Overview\">Overview</a>
  provides an overview of the Modelica Standard Library
  inside the <a href=\"modelica://Modelica.UsersGuide\">User's Guide</a>.</li>
<li><a href=\"modelica://Modelica.UsersGuide.ReleaseNotes\">Release Notes</a>
 summarizes the changes of new versions of this package.</li>
<li> <a href=\"modelica://Modelica.UsersGuide.Contact\">Contact</a>
  lists the contributors of the Modelica Standard Library.</li>
<li> The <b>Examples</b> packages in the various libraries, demonstrate
  how to use the components of the corresponding sublibrary.</li>
</ul>

<p>
This version of the Modelica Standard Library consists of
</p>
<ul>
<li><b>1600</b> models and blocks, and</li>
<li><b>1350</b> functions</li>
</ul>
<p>
that are directly usable (= number of public, non-partial classes). It is fully compliant
to <a href=\"https://www.modelica.org/documents/ModelicaSpec32Revision2.pdf\">Modelica Specification Version 3.2 Revision 2</a>
and it has been tested with Modelica tools from different vendors.
</p>

<p>
<b>Licensed by the Modelica Association under the Modelica License 2</b><br>
Copyright &copy; 1998-2016, ABB, AIT, T.&nbsp;B&ouml;drich, DLR, Dassault Syst&egrave;mes AB, Fraunhofer, A.&nbsp;Haumer, ITI, C.&nbsp;Kral, Modelon,
TU Hamburg-Harburg, Politecnico di Milano, XRG Simulation.
</p>

<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>

<p>
<b>Modelica&reg;</b> is a registered trademark of the Modelica Association.
</p>
</html>"));
end Modelica;

package Supermarktmodelle_342
  import PI = Modelica.Constants.pi;
  import SI = Modelica.SIunits;
  import NonSI = Modelica.SIunits.Conversions.NonSIunits;
  import g = Modelica.Constants.g_n;

  package Components

    package Compressor

      model SimplifiedRack "Simlified Rack"
        extends
        TIL.VLEFluidComponents.Compressors.BaseClasses.PartialEffCompressor;

        parameter Real C=1 "maximum";
        parameter Real K=0.1 "linear decrease";
        parameter Real volumetricEfficiency=0.7;
        Real pi;
        Real eta;
      equation
        pi = portB.p/portA.p;

        eta = C - K*pi;

        volEff = volumetricEfficiency;
        isEff = eta;
        effIsEff = eta;
      end SimplifiedRack;
    end Compressor;

    package Gascooler

      package Geometry

        record Kl_Sup_WRG_84_119EC28V "Kl_Sup_WRG_84_119EC28V"
          extends TIL.HeatExchangers.FinAndTube.Geometry.FinAndTubeGeometry(
            finnedTubeLength=2.7,
            nSerialTubes=3,
            serialTubeDistance=0.025,
            nParallelTubes=48,
            parallelTubeDistance=0.024,
            finThickness=1e-5,
            finPitch=0.0021,
            tubeInnerDiameter=7.24e-3,
            tubeWallThickness=1.5e-3,
            nTubeSideParallelHydraulicFlows=16);
        end Kl_Sup_WRG_84_119EC28V;
      end Geometry;
    end Gascooler;
  end Components;

  package Extremum_Seeking_Control
    extends TIL.Internals.ClassTypes.ExamplePackage;

    package AlgebraischAnsatz2

      model FanConstantEta
        "Fan with quadratic characteristic and consideration of power losses"
        extends TIL.GasComponents.Fans.BaseClasses.PartialFan(
          orientation="A",
          summary(P_loss=P_loss),
          dp(start=dpInitial),
          volumeFlowRate(start=V_flow_Start));

        /********************************************************/
        /*         Connectors & Gas-Object                      */
        /********************************************************/

        TIL.Connectors.RotatoryFlange rotatoryFlange if use_mechanicalPort
          annotation (Placement(transformation(extent={{-90,-10},{-70,10}},
                rotation=0)));

      protected
        TIL.Internals.GetInputsRotary getInputsRotary annotation (Placement(
              transformation(extent={{-34,-10},{-14,10}}, rotation=0)));

        /********************************************************/
        /*                      Variables                       */
        /********************************************************/

      public
        SI.Power P_loss "Overall power losses";

        SI.Frequency n "Speed";

      protected
        SI.AngularVelocity w "Angular velocity";
        SI.AngularAcceleration a "Angular acceleration";
        SI.Torque fluidTorque "Fluid torque";

        SI.Density d_gas "Actual gas density in fan";

        //_______  Characteristic Curve Variables ______________

        final parameter TIL.GasComponents.Fans.Internals.Coefficients coef=
            TIL.GasComponents.Fans.Internals.calcCoefficients(
                  V_flow_nominal,
                  dp_nominal,
                  V_flow0,
                  deltaV_flow);

        //__________________ Affinity Law Variables __________________

        TIL.GasComponents.Fans.Internals.Coefficients coef_affinity;

        /********************************************************/
        /*                      Parameters                      */
        /********************************************************/

        //____________________   Mechanics   _____________________
      public
        parameter Boolean use_mechanicalPort=false
          "= true, if mechanical port is used";

        parameter Boolean steadyStateMomentum=false
          "true, to avoid differentiation of angular speed"
          annotation (Dialog(enable=use_mechanicalPort));

        parameter SI.Inertia J=5e-3 "Moment of inertia of the fan"
          annotation (Dialog(enable=use_mechanicalPort));

        parameter SI.Frequency nFixed(min=0) = 50 "Constant speed"
          annotation (Dialog(enable=not use_mechanicalPort));

        //____________________ Energy balance _____________________

        parameter Boolean isenthalpicProcess=false "= true, if h_out = h_in"
          annotation (Dialog(tab="Advanced", group="Energy balance"));

        parameter SI.Temperature maxDeltaT(displayUnit="K") = 5
          "Maximal possible temperature difference from portA to portB, when m_flow is almost zero."
          annotation (Dialog(tab="Advanced", group="Energy balance"));

      protected
        final parameter Real dh_dhmax_transition=0.9
          "Transition point in percentage of maximal possible enthalpy difference from portA to portB."
          annotation (Dialog(tab="Advanced", group="Energy balance"));

        SI.SpecificEnthalpy dh "Enthalpy difference from portA to portB.";

        //__________________   Fan characteristic   __________________
      public
        parameter SI.Frequency n_nominal=50 "Nominal value for fan speed"
          annotation (Dialog(group=
                "Fan Characteristic @ nominal gas conditions"));

        parameter SI.Pressure dp_nominal(displayUnit="Pa") = 360
          "Nominal value for pressure increase @ n_nominal" annotation (Dialog(
              group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.VolumeFlowRate V_flow_nominal=0.1
          "Nominal value for volume flow rate @ n_nominal" annotation (Dialog(
              group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.VolumeFlowRate V_flow0=0.21
          "Volume flow rate @ dp = 0 and @ n_nomial" annotation (Dialog(group=
                "Fan Characteristic @ nominal gas conditions"));

        parameter SI.VolumeFlowRate deltaV_flow(min=0) = 0.07
          "Volume flow rate difference between V_flow0 and gradient line @ nominal point"
          annotation (Dialog(group=
                "Fan Characteristic @ nominal gas conditions"));

        parameter SI.Temperature T_nominal=298.15
          "Nominal value for gas temperature" annotation (Dialog(group=
                "Nominal gas conditions used for Fan Characteristic"));

        parameter SI.Pressure p_nominal=1.013e5
          "Nominal value for gas pressure" annotation (Dialog(group=
                "Nominal gas conditions used for Fan Characteristic"));

        parameter Real[gasType.nc] mixingRatio_nominal=gasType.defaultMixingRatio
          "Nominal value for gas mixing ratio" annotation (Dialog(group=
                "Nominal gas conditions used for Fan Characteristic"));

      protected
        SI.HeatFlowRate Q_flow_loss "Heat flow rate of fan losses to ambient";

        //____________________   Losses an Efficiencies   _____________________
      public
        parameter SI.Efficiency eta=0.3
          "Fan efficiency @ maximum hydraulic power and nominal conditions => V_flow_maxPhyd"
          annotation (Dialog(tab="Fan Power",group=
                "Losses and Efficiencies @ nominal conditions"));

        //____________________ Gas Properties _____________________
      protected
        final parameter SI.Density d_nominal(start=1)=
          TILMedia.GasFunctions.density_pTxi(
                gasType,
                p_nominal,
                T_nominal,
                mixingRatio_nominal) annotation (Evaluate=true);

        /********************************************************/
        /*                  Initialization                      */
        /********************************************************/
      public
        parameter SI.Pressure dpInitial=500
          "Initialization of pressure inrease" annotation (Dialog(tab=
                "Start Values and Initialization", group="Initialization"));

        parameter SI.VolumeFlowRate V_flow_Start=1
          "Start value for volume flow rate" annotation (Dialog(tab=
                "Start Values and Initialization", group="Start Value"));

      equation
        assert(V_flow_nominal < V_flow0,
          "Parameter V_flow_nominal has to be smaller than V_flow0.");

        d_gas = gas.d;

        portA.m_flow = volumeFlowRate*d_gas;

        //____________________ Affinity laws _____________________

        //  V_flow_affinity = V_flow_nominal *n/n_nominal;
        //  dp_affinity = dp_nominal *(n/n_nominal)^2*(d_gas/d_nominal);

        coef_affinity.dp_0 = coef.dp_0*d_gas/d_nominal*(n/n_nominal)^2;
        coef_affinity.c1 = coef.c1*d_gas/d_nominal*n/n_nominal;
        coef_affinity.c2 = coef.c2*d_gas/d_nominal;
        coef_affinity.V_flow_transition = coef.V_flow_transition*n/n_nominal;

        //____________________ Fan characteristic _____________________

        // Equation without mathematical optimization for modelica:
        // quadratic:    dp = V_flow * (c2*V_flow + c1) + dp_0
        // or linear:    dp = dp_transition + (V_flow - V_flow_transition)*ddp_dV_flow_transition

        dp = TIL.GasComponents.Fans.Internals.semiSquareFunction(volumeFlowRate,
          coef_affinity);

        //____________________ Power calculation _____________________

        P_loss = P_hyd*(1/eta - 1);

        P_hyd = dp*volumeFlowRate;

        P_shaft = P_loss + P_hyd;

        //____________________ Mechanics ___________________________

        if use_mechanicalPort then
          der(getInputsRotary.rotatoryFlange.phi) = w;
          J*a + fluidTorque + getInputsRotary.rotatoryFlange.tau = 0
            "Momentum balance";
        else
          n = nFixed;
          getInputsRotary.rotatoryFlange.phi = 0.0;
        end if;

        w = 2*PI*n;

        if (steadyStateMomentum) then
          a = 0;
        else
          a = der(w);
        end if;

        fluidTorque = if noEvent(w < 1e-8) then 0 else P_shaft/w;

        //____________________ Balance equations ___________________

        // Energy balance
        if isenthalpicProcess then
          portB.h_outflow = inStream(portA.h_outflow);
          portA.h_outflow = inStream(portB.h_outflow);
        else
          portB.h_outflow = inStream(portA.h_outflow) + dh;
          portA.h_outflow = inStream(portB.h_outflow) + dh;
        end if;
        // If mass flow rate is near zero a heat flow rate "to ambient" (Q_flow_loss) is implemented.
        dh = TIL.Utilities.Numerics.smoothLimiter(P_loss/noEvent(max(abs(portA.m_flow),
          1e-12)), maxDeltaT*gas.cp);
        Q_flow_loss = dh*abs(portA.m_flow) - P_loss;

        // Mass and Momentum balance see partial class

        //_____________________ Connections ________________________

        connect(rotatoryFlange, getInputsRotary.rotatoryFlange)
          annotation (Line(points={{-80,0},{-34,0}}, color={135,135,135}));

        annotation (Images(Dialog(
              tab="General",
              group="Fan Characteristic @ nominal gas conditions",
              source="./Images/FanCharacteristic.png"), Dialog(
              tab="Fan Power",
              group="Losses and Efficiencies @ nominal conditions",
              source="./Images/FanPowerLosses.png")), Documentation(info="<html>
        <br>
        <table border=1 cellspacing=0 cellpadding=3>
        <tr>
        <th colspan=2>Model overview</th>
        </tr>
        <tr>
        <td>mass balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>energy balance:</td><td>steady state</td>
        </tr>
        <tr>
        <td>differential states:</td><td>-</td>
        </tr>
        <tr>
        <td>momentum equation:</td><td>pressure increase</td>
        </tr>
        </table>
        <p>
        In order to parameterize the second order fan model the following parameters are needed:
        </p>
        <ul>
            <li> n_nominal - nominal speed </li>
            <li> dp_nominal - pressure increase @ V_flow_nominal </li>
            <li> V_flow_nominal - volume flow rate @ dp_nominal </li>
            <li> V_flow0 - volume flow rate @ dp = 0 </li>
            <li> deltaV_flow - volume flow rate differenz between V_flow0 and gradient line @ nominal point </li>
        </ul>
        <br>
        <p>
        The fan characteristic is put together by a quadratic and linear function. 
        The following picture illustrates fan characteristic: 
        The pressure increase dependend on the volume flow rate at nominal speed and nominal density.
        </p>
        <br>
        <br>
        <img src=\"modelica://TIL/Images/FanCharacteristicInfo.png\" width=\"530\">
        <p>
        The nominal gas density is calculated with the following parameters:
        </p>
        <ul>
            <li> T_nominal - nominal gas temperature</li>
            <li> p_nominal - nominal gas pressure</li>
            <li> mixingRatio_nominal - nominal gas mixing ratio</li>
        </ul>
        <br>
        <p>
        The actual fan operation point is calculated by using the affinity laws.
        </p>
        <hr>
        <p>
        To calculate the shaft power the following parameters are needed:
        </p>
        <ul>
            <li> eta_maxPhyd - Fan efficiency @ maximum hydraulic power</li>
            <li> bladeLossExponent - Affects the blade loss</li>
            <li> impactLossCoefficient - Affects the impact loss</li>
        </ul>
        <br>
        <p>
        The following picture illustrates the fan power curves plotted over the volume flow rate. 
        </p>
        <br>
        <br>
        <br>
        <img src=\"modelica://TIL/Images/FanPowerInfo.png\" width=\"620\">
        <br>
        <p>
        The efficiency maximum is not necessarily identical with the fan efficiency at the maximum hydraulic power 'eta_maxPhyd'. 
        The variation and influence of the loss parameters is also shown in the picture.
        </p>
        <hr>
        </html>"));
      end FanConstantEta;
    end AlgebraischAnsatz2;
  end Extremum_Seeking_Control;

  package Paper_R744Regelung_Algebraisch_

    package Ventil_System

      model Gleichungen "Supermarket Refrigeration System CO2"
        import TIL3_AddOn_Supermarkt_Wurm = Supermarktmodelle_342;

        //Systemeinstellungen
        parameter Boolean usedgc_aus_Tdp=true;

        parameter Real etaLufter_max=0.35 "Maximum Efficiency"
         annotation(Dialog(tab="System Settings",group="Fan Characteristics"));
        parameter Real zeta=0.7 "Pressure loss coefficient"
        annotation(Dialog(tab="System Settings",group="Fan Characteristics"));
        parameter Real hydrDiam=0.8 "Hydraulic diameter of each fan"
        annotation(Dialog(tab="System Settings",group="Fan Characteristics"));
        parameter Real TWallStart=273.15 + 10 "Initialize wall temperature gascooler"
          annotation(Dialog(tab="Initialization"));
        parameter Real C_Compressor=0.86 "Maximum isentropic efficiency"
          annotation(Dialog(tab="System Settings",group="Compressor Characteristics"));
        parameter Real K_Compressor=0.1 "Coefficient of linear drop of efficiency"
            annotation(Dialog(tab="System Settings",group="Compressor Characteristics"));
        parameter Integer numberFan=2 "Number of Fans"
        annotation(Dialog(tab="System Settings",group="Fan Characteristics"));
        parameter Real pHochdruckStart=50e5 "Initialize phigh"
          annotation(Dialog(tab="Initialization"));

        //Dampfdruckkurve fr pmin
        TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType;
        TILMedia.VLEFluidObjectFunctions.VLEFluidPointer vleFluidPointer=
            TILMedia.VLEFluidObjectFunctions.VLEFluidPointer(
                  vleFluidType.concatVLEFluidName,
                  7,
                  vleFluidType.mixingRatio_propertyCalculation[1:end - 1]/sum(
              vleFluidType.mixingRatio_propertyCalculation),
                  vleFluidType.nc_propertyCalculation,
                  vleFluidType.nc,
                  0);
        SI.MassFraction xi[vleFluidType.nc - 1];
        Real pTUmgebung=TILMedia.VLEFluidObjectFunctions.bubblePressure_Txi(
                  min(TUmgebung, 273.15 + 31),
                  xi,
                  vleFluidPointer);
        Real pmin;

        //Fr Paper
        Real dCOPTotaldp_Formel;
        Real dCOPTotaldp_Formel_Synonym;
        Real dCOPTotaldn_Formel;
        Real dCOPTotaldn_Formel_Synonym;

        //Wichtigste Ergebnisse
        Real Res_COPTotal=Qdotcool/(CompressorPower + FanPower);
        Real Res_deltaT=gc_aus_T - TUmgebung;
        Real Res_COPKompressor=Qdotcool/CompressorPower;
        Real Res_nfan=nLufter;
        Real Res_phigh=phigh;

        //----------------------------------------------------------------------------
        //Allgemein
        Real Qdotcool=m_flow*(verd_ein_h - gc_aus_h);
        Real CompressorPower=m_flow*(verd_aus_h - verd_ein_h);
        //----------------------------------------------------------------------------
        //Optimaler Hochdruck - Allgemein
        Real dCOPTotaldp=(ustrichp*v - vstrichp*u)/v^2;
        Real u=Qdotcool;
        Real v=CompressorPower + FanPower;
        Real ustrichp=dmdotdp*(verd_ein_h - gc_aus_h) + m_flow*(-dgc_aus_hdp);
        //eigentlich 0
        Real vstrichp=dmdotdp*(verd_aus_h - verd_ein_h) + m_flow*(dgc_ein_hdp
             - dverd_ein_hdp) + dFanPowerdp;

        //Optimaler Hochdruck - Ausgang nach Gaskhler
        Real dgc_aus_hdp=gc_aus_T*(dsdp + dsdT*dgc_aus_Tdp) + 1/gc_aus_d;
        Real dsdp=-gc_aus_beta/gc_aus_d;
        //wichtig fr Vernderung der Enthalpie entlang der Isothermen
        Real dsdT=gc_aus_cp/gc_aus_T;
        //Bridgeman; --> wichtig fr Einfluss der sinkenden Temperatur bei DruckerhÃ¶hung
        Real dgc_aus_Tdp;
        Real dgc_aus_Tdp_intern=(gc_aus_T - TUmgebung)*factor_pressure/(max(
            1e-3, Res_phigh - pmin));

        //Optimaler Hochdruck - Verdichtereintritt und Gaskhlereintritt
        Real dverd_ein_hdp=0;
        //Gleichung 6
        Real dgc_ein_hdp=(verd_aus_h - verd_ein_h)/(Res_phigh - plow);

        //Optimaler Hochdruck - Massenstrme
        Real dmdotdp=(Qdotcool*dgc_aus_hdp)/((verd_ein_h - gc_aus_h)^2);
        // Gleichung 10

        //Optimaler Hochdruck - Sonstiges
        Real dFanPowerdp=0;
        Real factor_pressure;

        //-----------------

        //----------------------------------------------------------------------------
        //----------------------------------------------------------------------------

        //Optimale Geblsedrehzahl - Allgemein
        Real dCOPTotaldn=(ustrichdn*v - vstrichdn*u)/v^2;
        //Gleichung 18
        Real ustrichdn=dmdotdn*(verd_ein_h - gc_aus_h) - m_flow*dgc_aus_hdn;
        Real vstrichdn=dmdotdn*(verd_aus_h - verd_ein_h) + dFanPowerdn;

        //Optimale Geblsedrehzahl - Ausgang nach Gaskhler
        Real dgc_aus_hdn=gc_aus_cp*dgc_aus_Tdn;
        //Gleichung 20
        Real dgc_aus_Tdn=factor_speed*(gc_aus_T - TUmgebung)/(Res_nfan);

        //Optimale Geblsedrehzahl - Massenstrme
        Real dmdotdn=dgc_aus_hdn*Qdotcool/((verd_ein_h - gc_aus_h)^2);

        //Optimale Geblsedrehzahl - Sonstiges
        Real dFanPowerdn=(FanPower)/nLufter*3;
        Real factor_speed;

        //----------------------------------------------------------------------------
        //Messwerte - Gaskhleraustritt
        Real gc_aus_h;
        Real gc_aus_T;
        Real gc_aus_cp;
        Real gc_aus_d;
        Real gc_aus_beta;

        //Messwerte - Gaskhlereintritt
        Real gc_ein_cp;
        Real gc_ein_h;
        Real phigh;

        //Messwerte - Sonstiges
        Real FanPower;
        Real nLufter;

        Real plow;
        Real verd_ein_h;
        Real verd_aus_h;

        //Messwerte - Massenstrme
        Real m_flow;
        //----------------------------------------------------------------------------

        Modelica.Blocks.Interfaces.RealInput Qdotkaelte annotation (Placement(
              transformation(extent={{-360,-100},{-320,-60}})));
        Modelica.Blocks.Interfaces.RealInput TUmgebung annotation (Placement(
              transformation(extent={{-360,228},{-320,268}})));
        Modelica.Blocks.Interfaces.RealOutput yTGCCelsius
          annotation (Placement(transformation(extent={{290,-92},{330,-52}})));
        Modelica.Blocks.Interfaces.RealOutput ydeltaT
          annotation (Placement(transformation(extent={{292,72},{332,112}})));
        Modelica.Blocks.Interfaces.RealOutput yTGC
          annotation (Placement(transformation(extent={{288,228},{328,268}})));
        Modelica.Blocks.Sources.RealExpression realExpression1(y=gc_aus_T -
              273.15)
          annotation (Placement(transformation(extent={{250,-82},{270,-62}})));
        Modelica.Blocks.Sources.RealExpression realExpression2(y=gc_aus_T -
              TUmgebung)
          annotation (Placement(transformation(extent={{254,82},{274,102}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=gc_aus_T)
          annotation (Placement(transformation(extent={{248,238},{268,258}})));
      equation
        factor_speed = 0.1082*(TUmgebung - 273.15) - 5.956;
        factor_pressure = 0.1396*(TUmgebung - 273.15) - 6.41;
        pmin = 1.32e5*(TUmgebung - 273.15) + 31.4e5;
        //Approximation von pmin
        /*
  if TUmgebung<(273.15+35) then //Unterkritisch
    pmin=pTUmgebung;
  else
    pmin=2.6*(TUmgebung-273.15)*1e5-15e5;
  end if;
  */
        dCOPTotaldp_Formel = ((gc_aus_T*gc_aus_beta - 1)/gc_aus_d - gc_aus_cp*
          factor_pressure*(gc_aus_T - TUmgebung)/(max(1e-3, (Res_phigh - pmin)))
           - (verd_ein_h - gc_aus_h)/(Res_phigh - plow))*(CompressorPower^2)/((
          gc_ein_h - verd_ein_h)*(CompressorPower + FanPower)^2);
        dCOPTotaldn_Formel = -CompressorPower/(max(0.1, (CompressorPower +
          FanPower)^2*Res_nfan*(gc_ein_h - verd_ein_h)))*(CompressorPower*
          gc_aus_cp*factor_speed*(gc_aus_T - TUmgebung) + 3*FanPower*(
          verd_ein_h - gc_aus_h));
        dCOPTotaldp_Formel_Synonym = (gc_aus_T*gc_aus_beta - 1)/gc_aus_d -
          gc_aus_cp*factor_pressure*(gc_aus_T - TUmgebung)/(max(1e-3, (
          Res_phigh - pmin))) - (verd_ein_h - gc_aus_h)/(Res_phigh - plow);
        dCOPTotaldn_Formel_Synonym = (CompressorPower*gc_aus_cp*factor_speed*(
          gc_aus_T - TUmgebung) + 3*FanPower*(verd_ein_h - gc_aus_h));

        //----------------------
        if usedgc_aus_Tdp then
          dgc_aus_Tdp = dgc_aus_Tdp_intern;
        else
          dgc_aus_Tdp = 0;
        end if;
        connect(realExpression1.y, yTGCCelsius) annotation (Line(
            points={{271,-72},{284,-72},{284,-72},{310,-72}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(realExpression2.y, ydeltaT) annotation (Line(
            points={{275,92},{312,92}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(realExpression.y, yTGC) annotation (Line(
            points={{269,248},{308,248}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-340,-100},{300,300}},
              initialScale=0.1)),
          experiment(StopTime=1.3e5, __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput(equdistant=false),
          Icon(coordinateSystem(
              extent={{-340,-100},{300,300}},
              preserveAspectRatio=false,
              initialScale=0.1)),
          Documentation(info="<html>
        <p>
        This CO2-Booster system represents a typical cycle for a CO2-Supermarket refrigeration plant.
        The R744 refrigeration cycle has two evaporators and there are four pressure levels in total.
        In the low temperature cabinet, the refrigerant evaporates at -28&deg;C and 12 bar.
        In the low pressure compressors, the refrigerant is compressed to the evaporation pressure of the medium temperature cabinets.
        The refrigerant mass flow from both cabinets are mixed and compressed to the condensing pressure.
        After throtteling in the HP-Valve, the liquid refrigerant is separated in a medium pressure receiver and afterwards throttled further down to the evaporation pressures.
        </p>
        </html>"));
      end Gleichungen;
    end Ventil_System;
  end Paper_R744Regelung_Algebraisch_;

  package Identification_Physical_System

    model Kreislauf "Supermarket Refrigeration System CO2"
      extends
      Supermarktmodelle_342.Paper_R744Regelung_Algebraisch_.Ventil_System.Gleichungen;

      TIL.HeatExchangers.FinAndTube.MoistAirVLEFluid.CrossFlowHX MTCabinet(
        redeclare model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum,
        vleFluidType=sim.vleFluidType1,
        initVLEFluid="linearEnthalpyDistribution",
        m_flowMoistAirStart=1,
        redeclare model FinSidePressureDropModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSidePressureDrop.ZeroPressureDrop,
        alphaAInitialVLEFluid=2980,
        pressureStateID=5,
        redeclare model WallMaterial = TILMedia.SolidTypes.TILMedia_Copper,
        hInitialVLEFluid_Cell1=193e3,
        hInitialVLEFluid_CellN=444e3,
        hxGeometry(
          finnedTubeLength=2,
          nTubeSideParallelHydraulicFlows=5,
          nSerialTubes=40,
          nParallelTubes=40),
        redeclare model TubeSidePressureDropModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop,
        redeclare model TubeSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ConstantAlpha
            (constantAlpha=6000),
        redeclare model FinSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.ConstantAlpha
            (constantAlpha=100),
        nCells=5,
        pVLEFluidStart=2800000,
        TInitialWall(displayUnit="K") = 273.15 + 4) annotation (Placement(
            transformation(
            origin={-62,-20},
            extent={{14,-14},{-14,14}},
            rotation=180)));

      TIL.VLEFluidComponents.Valves.OrificeValve MTValve(
          effectiveFlowAreaFixed=0.2e-5, use_effectiveFlowAreaInput=true)
        annotation (Placement(transformation(
            extent={{-8.0,-4.0},{8.0,4.0}},
            rotation=0,
            origin={-160,-20})));
      TIL.GasComponents.Volumes.Volume volume_MTCabinet(
        m_flowStart=5,
        volume=185,
        TInitial=280.15) annotation (Placement(transformation(
            extent={{-4.0,-8.0},{4.0,8.0}},
            rotation=90,
            origin={-66,-72})));
      TIL.OtherComponents.Thermal.HeatBoundary
        heatBoundaryWithInputs_MTCabinet_Einzeln(
        TFixed(displayUnit="K") = 273.15 + 21,
        boundaryType="Q_flow",
        Q_flowFixed(displayUnit="kW") = -85000,
        use_heatFlowRateInput=true) annotation (Placement(transformation(
            extent={{-4.0,-6.0},{4.0,6.0}},
            rotation=0,
            origin={-82,-72})));
      TIL.OtherComponents.Controllers.PIController PI_MTValve(
        yMin=0.2e-12,
        yMax=0.1e-3,
        controllerType="PI",
        initType="initialOutput",
        invertFeedback=true,
        yInitial=0.68e-5,
        activationTime=500,
        Ti=200,
        k=1e-7) annotation (Placement(transformation(
            origin={-144,12},
            extent={{-6,6},{6,-6}},
            rotation=180)));
      Modelica.Blocks.Sources.RealExpression setPoint_MT_Valve(y=8)
        annotation (Placement(transformation(
            origin={-116,12},
            extent={{10,-10},{-10,10}},
            rotation=0)));
      TIL.VLEFluidComponents.PressureStateElements.PressureState
        pressureState_5(
        vleFluidType=sim.vleFluidType1,
        pressureStateID=5,
        pInitial(displayUnit="Pa") = 28.004e5,
        fixedInitialPressure=false) annotation (Placement(transformation(
            extent={{6.0,6.0},{-6.0,-6.0}},
            rotation=180,
            origin={-118,-20})));
      inner TIL.SystemInformationManager sim(
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType1,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType2,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType3,
        redeclare TILMedia.GasTypes.TILMedia_MoistAir gasType1,
        redeclare TILMedia.GasTypes.TILMedia_MoistAir gasType2,
        redeclare TILMedia.LiquidTypes.TILMedia_Glysantin_60 liquidType1)
        annotation (Placement(transformation(extent={{254,188},{274,208}},
              rotation=0)));
      Supermarktmodelle_342.Components.Compressor.SimplifiedRack compressor(
        displacement=0.000204204,
        nFixed=50,
        use_mechanicalPort=true,
        C=C_Compressor,
        K=K_Compressor) annotation (Placement(transformation(
            extent={{-8,-8},{8,8}},
            rotation=0,
            origin={80,156})));
      TIL.VLEFluidComponents.PressureStateElements.PressureState
        pressureState_2(
        vleFluidType=sim.vleFluidType1,
        pressureStateID=2,
        fixedInitialPressure=true,
        pInitial(displayUnit="Pa") = pHochdruckStart) annotation (Placement(
            transformation(
            extent={{-6.0,-6.0},{6.0,6.0}},
            rotation=0,
            origin={48,200})));
      TIL.VLEFluidComponents.JunctionElements.VolumeJunction
        junction_from_IHX(
        volume=1e-2,
        hInitial=451e3,
        pressureStateID=5) annotation (Placement(transformation(
            extent={{-4.0,-4.0},{4.0,4.0}},
            rotation=90,
            origin={78,62})));
      TIL.VLEFluidComponents.Valves.OrificeValve HPValve(
        mdotSmooth=0.0005,
        vleFluidType=sim.vleFluidType1,
        x(start=0.5),
        effectiveFlowAreaFixed=2e-6,
        use_effectiveFlowAreaInput=true) annotation (Placement(transformation(
            extent={{-8,4},{8,-4}},
            rotation=270,
            origin={-258,148})));
      TIL.VLEFluidComponents.Sensors.Sensor_p sensor_HP(
        useTimeConstant=true,
        tau=5,
        initialSensorValue=pressureState_2.pInitial) annotation (Placement(
            transformation(
            extent={{-4.0,-4.0},{4.0,4.0}},
            rotation=270,
            origin={-238,178})));
      TIL.VLEFluidComponents.Separators.IdealSeparator MPReciever(
        vleFluidType=sim.vleFluidType1,
        mdotStart=1,
        pressureStateID=3,
        initialFillingLevel=0.3,
        hStart=223e3,
        V(displayUnit="l") = 0.4) annotation (Placement(transformation(
            extent={{-6.0,-10.0},{6.0,10.0}},
            rotation=0,
            origin={-200,58})));
      TIL.VLEFluidComponents.Valves.OrificeValve RecValve(
          effectiveFlowAreaFixed=0.9e-5, use_effectiveFlowAreaInput=true)
        annotation (Placement(transformation(
            extent={{-8.0,-4.0},{8.0,4.0}},
            rotation=0,
            origin={-120,62})));
      TIL.VLEFluidComponents.PressureStateElements.PressureState
        pressureState_3(pInitial(displayUnit="Pa") = 34.9e5, pressureStateID=
            3) annotation (Placement(transformation(
            extent={{-6.0,-6.0},{6.0,6.0}},
            rotation=90,
            origin={-260,118})));
      TIL.VLEFluidComponents.Sensors.Sensor_p sensor_MP(useTimeConstant=true,
          initialSensorValue=3490000) annotation (Placement(transformation(
            extent={{-4.0,-4.0},{4.0,4.0}},
            rotation=0,
            origin={-170,78})));
      TIL.OtherComponents.Controllers.PIController PI_RecValve(
        yMin=1e-10,
        yMax=1e-1,
        k=1e-8,
        Ti=10,
        controllerType="PI",
        initType="initialOutput",
        yInitial=1.02e-5,
        invertFeedback=true) annotation (Placement(transformation(extent={{-176,
                100},{-164,112}}, rotation=0)));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint5(stateViewerIndex=
            5) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=90,
            origin={-272,98})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint2(stateViewerIndex=
            2) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=90,
            origin={66,144})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint3(stateViewerIndex=
            3)
        annotation (Placement(transformation(extent={{26,206},{34,214}})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint10(stateViewerIndex=
           10)
        annotation (Placement(transformation(extent={{-144,72},{-136,80}})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint7(stateViewerIndex=
            7) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-130,-8})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint1(stateViewerIndex=
            1) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=90,
            origin={68,42})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint6(stateViewerIndex=
            6) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-190,10})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint9(stateViewerIndex=
            9) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-80,76})));
      TIL.OtherComponents.Controllers.PIController PI_Fan_MT(
        yMin=0.00001,
        controllerType="PI",
        initType="initialOutput",
        invertFeedback=true,
        k=0.1,
        Ti=100,
        yInitial=6,
        activationTime=3e3,
        yMax=50) annotation (Placement(transformation(extent={{-118,-52},{-106,
                -64}}, rotation=0)));
      Modelica.Blocks.Sources.RealExpression setPoint_FanMTCab(y=273.15 + 3)
        annotation (Placement(transformation(extent={{-162,-68},{-142,-48}},
              rotation=0)));
      TIL.GasComponents.Sensors.Sensor_T sensor_T_MT(
        useTimeConstant=true,
        tau=5,
        initialSensorValue=276.15) annotation (Placement(transformation(
            extent={{-4.0,-4.0},{4.0,4.0}},
            rotation=90,
            origin={-82,-40})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint4(stateViewerIndex=
            4) annotation (Placement(transformation(extent={{-264,206},{-256,
                214}})));
      TIL.GasComponents.Fans.SimpleFan fan_MT(
        presetVariableType="m_flow",
        m_flowFixed=1,
        use_massFlowRateInput=true,
        eta=0.95)
        annotation (Placement(transformation(extent={{-70,-60},{-54,-44}})));
      TILMedia.VLEFluid_ph vle_gas_aus(
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=statePoint4.h,
        p=statePoint4.p) annotation (Placement(transformation(extent={{-234,
                206},{-214,226}})));
      TILMedia.VLEFluid_ph vle_Sammlereintritt(
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=statePoint5.h,
        p=statePoint5.p) annotation (Placement(transformation(extent={{-234,
                120},{-214,140}})));
      TIL.VLEFluidComponents.Sensors.Sensor_T sensor_T(useTimeConstant=false,
          initialSensorValue=283.15) annotation (Placement(transformation(
              extent={{-196,210},{-188,218}})));
      TIL.OtherComponents.Controllers.PIController PI_HPComp(
        controllerType="PI",
        initType="initialOutput",
        invertFeedback=true,
        Ti=10,
        k=70e-4,
        yMax=350,
        yMin=20,
        yInitial=100) annotation (Placement(transformation(extent={{122,152},
                {110,164}}, rotation=0)));
      Modelica.Blocks.Sources.RealExpression setPoint_HP_Comp(y=28e5)
        annotation (Placement(transformation(
            origin={140,158},
            extent={{10,-10},{-10,10}},
            rotation=0)));
      TIL.OtherComponents.Mechanical.RotatoryBoundary
        rotatoryBoundaryWithInputs(use_nInput=true, nFixed=40) annotation (
          Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=180,
            origin={100,156})));
      TIL.VLEFluidComponents.Sensors.Sensor_p sensor_MP2(
        useTimeConstant=true,
        tau=5,
        initialSensorValue=2800000) annotation (Placement(transformation(
            extent={{-4.0,-4.0},{4.0,4.0}},
            rotation=270,
            origin={88,144})));
      TILMedia.VLEFluid_ph x1(
        computeVLEAdditionalProperties=false,
        computeVLETransportProperties=false,
        computeTransportProperties=false,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=statePoint4.h - 100,
        p=3500000)
        annotation (Placement(transformation(extent={{98,238},{118,258}})));
      TILMedia.VLEFluid_ph x2(
        computeVLEAdditionalProperties=false,
        computeVLETransportProperties=false,
        computeTransportProperties=false,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=statePoint4.h + 100,
        p=3500000)
        annotation (Placement(transformation(extent={{140,242},{160,262}})));
      TILMedia.VLEFluid_pT x3(
        computeVLEAdditionalProperties=false,
        computeVLETransportProperties=false,
        computeTransportProperties=false,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        p=sensor_HP.sensorValue - 100,
        T=sensor_T.sensorValue)
        annotation (Placement(transformation(extent={{96,272},{116,292}})));
      TILMedia.VLEFluid_pT x4(
        computeVLEAdditionalProperties=false,
        computeVLETransportProperties=false,
        computeTransportProperties=false,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        p=sensor_HP.sensorValue + 100,
        T=sensor_T.sensorValue)
        annotation (Placement(transformation(extent={{132,272},{152,292}})));
      TILMedia.VLEFluid_ph x5(
        computeVLEAdditionalProperties=false,
        computeVLETransportProperties=false,
        computeTransportProperties=false,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=x3.h,
        p=3500000)
        annotation (Placement(transformation(extent={{166,270},{186,290}})));
      TILMedia.VLEFluid_ph x6(
        computeVLEAdditionalProperties=false,
        computeVLETransportProperties=false,
        computeTransportProperties=false,
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=x4.h,
        p=3500000)
        annotation (Placement(transformation(extent={{188,270},{208,290}})));
      Supermarktmodelle_342.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
        Fan1(
        use_mechanicalPort=true,
        V_flow_Start=15,
        V_flow_nominal=4,
        n_nominal=11.5,
        dp_nominal=60,
        V_flow0=6,
        deltaV_flow=1,
        eta=1)         annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-76,248})));
      TIL.GasComponents.Boundaries.Boundary boundary(
        boundaryType="p",
        use_temperatureInput=true,
        TFixed=303.15)
        annotation (Placement(transformation(extent={{-102,238},{-94,258}})));
      TIL.GasComponents.HydraulicResistors.HydraulicResistor
        hydraulicResistor(zeta_fixed=zeta, hydraulicDiameter=hydrDiam)
        annotation (Placement(transformation(extent={{-60,246},{-48,250}})));
      TIL.GasComponents.Boundaries.Boundary boundary1(boundaryType="p")
        annotation (Placement(transformation(
            extent={{-4,-10},{4,10}},
            rotation=90,
            origin={-22,222})));
      TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary3(
          use_nInput=true)
        annotation (Placement(transformation(extent={{-94,250},{-86,270}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder(y_start=10, T=400)
        annotation (Placement(transformation(extent={{-118,256},{-110,264}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder1(
        y_start=35e5,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        T=50)
        annotation (Placement(transformation(extent={{-304,96},{-284,116}})));
      TILMedia.VLEFluid_pT vle_gas_ausdT(
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        p=statePoint4.p,
        T=sensor_T.sensorValue + 0.1)
        annotation (Placement(transformation(extent={{-70,138},{-50,158}})));
      TILMedia.VLEFluid_ph vle_gas_ein(
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        h=statePoint3.h,
        p=statePoint3.p)
        annotation (Placement(transformation(extent={{4,142},{24,162}})));
      TILMedia.VLEFluid_pT vle_WRG(
        redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType,
        p=statePoint4.p,
        T=273.15 + 50)
        annotation (Placement(transformation(extent={{-28,146},{-8,166}})));
      Modelica.Blocks.Interfaces.RealInput Mitteldruck
        annotation (Placement(transformation(extent={{-360,86},{-320,126}})));
      TIL.HeatExchangers.FinAndTube.GasVLEFluid.CrossFlowHX Gascooler(
        gasCellFlowType="flow B-A",
        initVLEFluid="linearEnthalpyDistribution",
        redeclare model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum,
        m_flowVLEFluidStart=0.78,
        redeclare model WallMaterial = TILMedia.SolidTypes.TILMedia_Copper,
        redeclare
          Supermarktmodelle_342.Components.Gascooler.Geometry.Kl_Sup_WRG_84_119EC28V
          hxGeometry,
        redeclare model FinEfficiencyModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.ConstFinEfficiency,
        pressureStateID=2,
        hInitialVLEFluid_CellN=500e3,
        pVLEFluidStart=pressureState_2.pInitial,
        redeclare model TubeSidePressureDropModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop,
        cellOrientation="B",
        redeclare model WallHeatConductionModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.ConstantR
            (constantR=1e-8),
        TInitialWall=TWallStart,
        nCells=20,
        hInitialVLEFluid_Cell1=215e3,
        redeclare model TubeSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ConstantAlpha
            (constantAlpha=4e3),
        redeclare model FinSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.ConstantAlpha
            (constantAlpha=500))
        annotation (Placement(transformation(extent={{-36,186},{-8,214}})));

      TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary1(
          use_nInput=true)
        annotation (Placement(transformation(extent={{-94,220},{-86,240}})));
      TIL.GasComponents.HydraulicResistors.HydraulicResistor
        hydraulicResistor1(zeta_fixed=zeta, hydraulicDiameter=hydrDiam)
        annotation (Placement(transformation(extent={{-62,214},{-50,218}})));
      TIL.GasComponents.Boundaries.Boundary boundary2(
        boundaryType="p",
        use_temperatureInput=true,
        TFixed=303.15)
        annotation (Placement(transformation(extent={{-102,206},{-94,226}})));
      TIL.GasComponents.JunctionElements.VolumeJunction junction2(volume=1e-3,
          fixedInitialPressure=false) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=90,
            origin={-40,216})));
      Supermarktmodelle_342.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
        Fan2(
        use_mechanicalPort=true,
        V_flow_Start=15,
        V_flow_nominal=4,
        n_nominal=11.5,
        dp_nominal=60,
        V_flow0=6,
        deltaV_flow=1,
        eta=1)         annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-76,216})));
      TIL.VLEFluidComponents.Sensors.Sensor_superheating sensor_MT(
        useTimeConstant=true,
        initialSensorValue=8,
        tau=1) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=90,
            origin={66,94})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint8(stateViewerIndex=
            8) annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-8,-6})));
      TIL.VLEFluidComponents.Sensors.StatePoint statePoint11(stateViewerIndex=
           11)
        annotation (Placement(transformation(extent={{-228,70},{-220,78}})));
      TIL.VLEFluidComponents.Sensors.Sensor_subcooling sensor_subcooling
        annotation (Placement(transformation(extent={{-296,200},{-288,208}})));
    equation

      //----------------------------------------------------------------------------
      //Messwerte - Gaskühleraustritt
      gc_aus_h = vle_gas_aus.h;
      //ok
      gc_aus_T = vle_gas_aus.T;
      //ok
      gc_aus_cp = vle_gas_aus.cp;
      //ok
      gc_aus_d = vle_gas_aus.d;
      //ok
      gc_aus_beta = vle_gas_aus.beta;
      //ok

      //Messwerte - Gaskühlereintritt
      gc_ein_cp = vle_gas_ein.cp;
      ///ok
      gc_ein_h = vle_gas_ein.h;
      //ok
      phigh = statePoint3.p;
      //ok

      //Messwerte - Sonstiges
      nLufter = Fan1.n;
      //ok

      plow = statePoint1.p;
      //ok
      verd_ein_h = statePoint2.h;
      //ok
      verd_aus_h = vle_gas_ein.h;
      //ok

      //Messwerte - Massenströme
      m_flow = compressor.portA.m_flow;
      //ok

      //Messdaten - Leistung
      FanPower = numberFan*Fan1.summary.P_shaft;
      //Gleichung 4  //ok

      connect(heatBoundaryWithInputs_MTCabinet_Einzeln.heatPort,
        volume_MTCabinet.heatPort) annotation (Line(
          points={{-82,-72},{-72,-72}},
          color={204,0,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(MTValve.portB, pressureState_5.portB) annotation (Line(
          points={{-152,-20},{-124,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(pressureState_5.portA, MTCabinet.portA_vle) annotation (Line(
          points={{-112,-20},{-76,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_HP.port, HPValve.portA) annotation (Line(
          points={{-242,178},{-258,178},{-258,156}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(MPReciever.portGas, RecValve.portA) annotation (Line(
          points={{-195,62},{-128,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(pressureState_3.portB, MPReciever.portInlet) annotation (Line(
          points={{-260,112},{-260,62},{-205,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(HPValve.portB, pressureState_3.portA) annotation (Line(
          points={{-258,140},{-258,132},{-260,132},{-260,124}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_MP.port, MPReciever.portGas) annotation (Line(
          points={{-170,74},{-170,62},{-195,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint1.sensorPort, junction_from_IHX.portA) annotation (
          Line(
          points={{72,42},{78,42},{78,58}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint10.sensorPort, RecValve.portA) annotation (Line(
          points={{-140,72},{-140,62},{-128,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint5.sensorPort, MPReciever.portInlet) annotation (Line(
          points={{-268,98},{-260,98},{-260,62},{-205,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint7.sensorPort, MTValve.portB) annotation (Line(
          points={{-130,-12},{-130,-20},{-152,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_T_MT.sensorValue, PI_Fan_MT.u_m) annotation (Line(
          points={{-84,-40},{-112,-40},{-112,-52.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(setPoint_FanMTCab.y, PI_Fan_MT.u_s) annotation (Line(
          points={{-141,-58},{-117.6,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(MTCabinet.portA_gas, fan_MT.portB) annotation (Line(
          points={{-62,-34},{-62,-44}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(fan_MT.portA, volume_MTCabinet.portB) annotation (Line(
          points={{-62,-60},{-62,-68}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint4.sensorPort, HPValve.portA) annotation (Line(
          points={{-260,206},{-260,156},{-258,156}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_MP.sensorValue, PI_RecValve.u_m) annotation (Line(
          points={{-170,80.2},{-170,100.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(setPoint_MT_Valve.y, PI_MTValve.u_s) annotation (Line(
          points={{-127,12},{-138.4,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(PI_RecValve.y, RecValve.effectiveFlowArea_in) annotation (Line(
          points={{-163.6,106},{-120,106},{-120,67}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(PI_MTValve.y, MTValve.effectiveFlowArea_in) annotation (Line(
          points={{-150.4,12},{-160,12},{-160,-15}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(PI_Fan_MT.y, fan_MT.m_flow_in) annotation (Line(
          points={{-105.6,-58},{-72,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(MTCabinet.portB_gas, volume_MTCabinet.portA) annotation (Line(
          points={{-62,-6},{-62,0},{-22,0},{-22,-82},{-62,-82},{-62,-76}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(fan_MT.portB, sensor_T_MT.port) annotation (Line(
          points={{-62,-44},{-62,-40},{-78,-40}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint2.sensorPort, compressor.portA) annotation (Line(
          points={{70,144},{80,144},{80,148}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(rotatoryBoundaryWithInputs.n_in, PI_HPComp.y) annotation (Line(
          points={{104,156},{106,156},{106,158},{109.6,158}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(PI_HPComp.u_s, setPoint_HP_Comp.y) annotation (Line(
          points={{121.6,158},{129,158}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sensor_MP2.port, compressor.portA) annotation (Line(
          points={{84,144},{80,144},{80,148}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_MP2.sensorValue, PI_HPComp.u_m) annotation (Line(
          points={{90.2,144},{116,144},{116,152.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(compressor.rotatoryFlange, rotatoryBoundaryWithInputs.rotatoryFlange)
        annotation (Line(
          points={{88,156},{100,156}},
          color={135,135,135},
          thickness=0.5,
          smooth=Smooth.None));
      connect(boundary.port, Fan1.portA) annotation (Line(
          points={{-98,248},{-82,248}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Fan1.portB, hydraulicResistor.portA) annotation (Line(
          points={{-70,248},{-60,248}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint3.sensorPort, pressureState_2.portB) annotation (Line(
          points={{30,206},{30,200},{42,200}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_T.port, HPValve.portA) annotation (Line(
          points={{-192,210},{-192,200},{-258,200},{-258,156}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(firstOrder1.u, Mitteldruck) annotation (Line(
          points={{-306,106},{-340,106}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(firstOrder1.y, PI_RecValve.u_s) annotation (Line(
          points={{-283,106},{-175.6,106}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(MPReciever.portLiquid, MTValve.portA) annotation (Line(
          points={{-200,48},{-200,-20},{-168,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(pressureState_2.portB, Gascooler.portB_vle) annotation (Line(
          points={{42,200},{-8,200}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Gascooler.portA_vle, HPValve.portA) annotation (Line(
          points={{-36,200},{-258,200},{-258,156}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Gascooler.portA_gas, boundary1.port) annotation (Line(
          points={{-22,214},{-22,222}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Fan1.rotatoryFlange, rotatoryBoundary3.rotatoryFlange)
        annotation (Line(
          points={{-76,254},{-76,260},{-90,260}},
          color={135,135,135},
          thickness=0.5,
          smooth=Smooth.None));
      connect(boundary2.T_in, boundary.T_in) annotation (Line(
          points={{-102,210},{-110,210},{-110,242},{-102,242}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(Fan2.portB, hydraulicResistor1.portA) annotation (Line(
          points={{-70,216},{-62,216}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Fan2.portA, boundary2.port) annotation (Line(
          points={{-82,216},{-98,216}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Fan2.rotatoryFlange, rotatoryBoundary1.rotatoryFlange)
        annotation (Line(
          points={{-76,222},{-76,230},{-90,230}},
          color={135,135,135},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint6.sensorPort, MTValve.portA) annotation (Line(
          points={{-194,10},{-200,10},{-200,-20},{-168,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(junction2.portC, hydraulicResistor.portB) annotation (Line(
          points={{-40,220},{-40,248},{-48,248}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(hydraulicResistor1.portB, junction2.portB) annotation (Line(
          points={{-50,216},{-44,216}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(junction2.portA, Gascooler.portB_gas) annotation (Line(
          points={{-40,212},{-40,180},{-22,180},{-22,186}},
          color={255,153,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(Qdotkaelte, heatBoundaryWithInputs_MTCabinet_Einzeln.Q_flow_in)
        annotation (Line(
          points={{-340,-80},{-214,-80},{-214,-74},{-86,-74}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(boundary.T_in, TUmgebung) annotation (Line(
          points={{-102,242},{-212,242},{-212,248},{-340,248}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(rotatoryBoundary3.n_in, firstOrder.y) annotation (Line(
          points={{-94,260},{-109.6,260}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(rotatoryBoundary1.n_in, firstOrder.y) annotation (Line(
          points={{-94,230},{-104,230},{-104,260},{-109.6,260}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sensor_MT.sensorValue, PI_MTValve.u_m) annotation (Line(
          points={{63.8,94},{63.8,6.2},{-144,6.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(junction_from_IHX.portA, MTCabinet.portB_vle) annotation (Line(
          points={{78,58},{78,-20},{-48,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(junction_from_IHX.portC, compressor.portA) annotation (Line(
          points={{78,66},{80,66},{80,148}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_MT.port, compressor.portA) annotation (Line(
          points={{70,94},{80,94},{80,148}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint8.sensorPort, MTCabinet.portB_vle) annotation (Line(
          points={{-8,-10},{-8,-20},{-48,-20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint11.sensorPort, MPReciever.portInlet) annotation (Line(
          points={{-224,70},{-224,62},{-205,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(compressor.portB, pressureState_2.portA) annotation (Line(
          points={{80,164},{80,200},{54,200}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(RecValve.portB, junction_from_IHX.portB) annotation (Line(
          points={{-112,62},{74,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(statePoint9.sensorPort, junction_from_IHX.portB) annotation (
          Line(
          points={{-80,72},{-80,62},{74,62}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_subcooling.port, HPValve.portA) annotation (Line(
          points={{-292,200},{-276,200},{-276,170},{-260,170},{-260,156},{-258,
              156}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-340,-100},{300,300}},
            initialScale=0.1), graphics={Text(
              extent={{172,264},{246,238}},
              lineColor={0,0,255},
              textString="Überprüfung dq/dh"), Text(
              extent={{216,292},{290,266}},
              lineColor={0,0,255},
              textString="Überprüfung dq/dp")}),
        experiment(StopTime=1.3e5, __Dymola_Algorithm="Dassl"),
        __Dymola_experimentSetupOutput(equdistant=false),
        Icon(coordinateSystem(
            extent={{-340,-100},{300,300}},
            preserveAspectRatio=false,
            initialScale=0.1)),
        Documentation(info="<html>
        <p>
        This CO2-Booster system represents a typical cycle for a CO2-Supermarket refrigeration plant.
        The R744 refrigeration cycle has two evaporators and there are four pressure levels in total.
        In the low temperature cabinet, the refrigerant evaporates at -28&deg;C and 12 bar.
        In the low pressure compressors, the refrigerant is compressed to the evaporation pressure of the medium temperature cabinets.
        The refrigerant mass flow from both cabinets are mixed and compressed to the condensing pressure.
        After throtteling in the HP-Valve, the liquid refrigerant is separated in a medium pressure receiver and afterwards throttled further down to the evaporation pressures.
        </p>
        </html>"));
    end Kreislauf;

    model Inputs
      extends Kreislauf(
        Gascooler(redeclare model TubeSidePressureDropModel =
            TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop),
        PI_HPComp(yInitial=40, activationTime=2e3),
        setPoint_FanMTCab(y=278.15),
        PI_Fan_MT(yMax=25),
        firstOrder(T=30, y_start=20));

      Modelica.Blocks.Math.Gain k(k=1e-6) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-300,148})));
      Modelica.Blocks.Interfaces.RealInput nFan annotation (Placement(
            transformation(extent={{20,-20},{-20,20}},
            rotation=90,
            origin={-190,300})));
      Modelica.Blocks.Interfaces.RealInput Ventil annotation (Placement(
            transformation(
            extent={{20,-20},{-20,20}},
            rotation=90,
            origin={38,300})));
      Modelica.Blocks.Interfaces.RealOutput yHP
        annotation (Placement(transformation(extent={{290,16},{330,56}})));
      Modelica.Blocks.Sources.RealExpression realExpression3(y=Res_phigh/1e5)
        annotation (Placement(transformation(extent={{248,26},{268,46}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder2(
        initType=Modelica.Blocks.Types.Init.InitialOutput,
      T=10,
      y_start=5.5)
        annotation (Placement(transformation(extent={{-340,138},{-320,158}})));
    equation

      connect(firstOrder.u, nFan) annotation (Line(
          points={{-118.8,260},{-190,260},{-190,300}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(realExpression3.y, yHP)
        annotation (Line(points={{269,36},{310,36}}, color={0,0,127}));
    connect(HPValve.effectiveFlowArea_in, k.y) annotation (Line(points={{-263,
            148},{-276,148},{-289,148}}, color={0,0,127}));
    connect(k.u, firstOrder2.y)
      annotation (Line(points={{-312,148},{-319,148}}, color={0,0,127}));
    connect(firstOrder2.u, Ventil) annotation (Line(points={{-342,148},{-368,
            148},{-368,154},{-386,154},{-386,328},{38,328},{38,300}}, color={0,
            0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                {{-340,-100},{300,300}})));
    end Inputs;
  end Identification_Physical_System;
  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={1,1},
        initialScale=0.1), graphics={Bitmap(
          extent={{-100,100},{100,-100}},
          imageSource=
              "iVBORw0KGgoAAAANSUhEUgAAAgAAAAIACAYAAAD0eNT6AAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAZdEVYdFNvZnR3YXJlAEFkb2JlIEltYWdlUmVhZHlxyWU8AAADaGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4gPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS4wLWMwNjEgNjQuMTQwOTQ5LCAyMDEwLzEyLzA3LTEwOjU3OjAxICAgICAgICAiPiA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPiA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIiB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIgeG1sbnM6c3RSZWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZVJlZiMiIHhtbG5zOnhtcD0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wLyIgeG1wTU06T3JpZ2luYWxEb2N1bWVudElEPSJ4bXAuZGlkOkY3N0YxMTc0MDcyMDY4MTE5N0E1ODZBOEE1QTEwMDA3IiB4bXBNTTpEb2N1bWVudElEPSJ4bXAuZGlkOjg4RjQwNEY0NEU0QzExRTE5RjAxRjk3Q0VCNTBEMDI5IiB4bXBNTTpJbnN0YW5jZUlEPSJ4bXAuaWlkOjg4RjQwNEYzNEU0QzExRTE5RjAxRjk3Q0VCNTBEMDI5IiB4bXA6Q3JlYXRvclRvb2w9IkFkb2JlIFBob3Rvc2hvcCBDUzUuMSBNYWNpbnRvc2giPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDpCQkI2RDg3QjI2MjA2ODExOEE2REUwQzAzNjQwQzhCMCIgc3RSZWY6ZG9jdW1lbnRJRD0ieG1wLmRpZDpGNzdGMTE3NDA3MjA2ODExOTdB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"
               +
              "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",
          fileName="modelica://TIL 3.1.0/Images/TIL.png")}),
    uses(
      preferedView="info",
      Documentation(info="<html><br>
<img src=\"../Images/InfoTIL.png\"><br>
<hr>
</html>", revisions=""),
      Icon(Text(
          extent=[-120, 144; 120, 95],
          string="%name",
          style(color=1))),
      WurmKomponenten(version="1.0.5"),
      TIL3_AddOn_Cabin(version="3.1.3"),
      Modelica(version="3.2.2"),
      TILFileReader(version="3.5.1"),
      PI_SP(version="1"),
      TILMedia(version="3.4.1"),
      TIL(version="3.4.1")),
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
    version="1",
    conversion(noneFromVersion=""));
end Supermarktmodelle_342;

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
      Modelica.Blocks.Sources.RealExpression T_A(y=293.15)
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
      use_activeInput=true) annotation (Placement(transformation(
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
      startTime=50000,
      height=-1)
      annotation (Placement(transformation(extent={{-142,68},{-122,88}})));
    Modelica.Blocks.Sources.Step step_P(
      height=1,
      offset=0,
      startTime=100000)
      annotation (Placement(transformation(extent={{-144,32},{-124,52}})));
    Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=true, startTime=
          50000)
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
            58.4,16.8},{58.4,16},{106,16}},   color={0,0,127}));
      connect(Physical_System.yHP, p_H) annotation (Line(points={{35,-4.4},{
              64.5,-4.4},{64.5,-4},{106,-4}}, color={0,0,127}));
      connect(Physical_System.TUmgebung, T_A.y) annotation (Line(points={{-30,
              16.8},{-54,16.8},{-54,16},{-79,16}}, color={0,0,127}));
      connect(Physical_System.Mitteldruck, P_M.y) annotation (Line(points={{-30,
              2.6},{-54,2.6},{-54,0},{-79,0}}, color={0,0,127}));
      connect(Physical_System.Qdotkaelte, Q_C.y)
        annotation (Line(points={{-30,-16},{-79,-16}}, color={0,0,127}));
    connect(Physical_System.yTGC, multivariable_Controller.u_m[1]) annotation (
        Line(points={{34.8,16.8},{46,16.8},{46,94},{18,94},{-6.2,94},{-6.2,88},
            {-6.2,81.9}}, color={0,0,127}));
    connect(Physical_System.yHP, multivariable_Controller.u_m[2]) annotation (
        Line(points={{35,-4.4},{38,-4.4},{38,94},{24,94},{-6.2,94},{-6.2,86},{
            -6.2,79.9}}, color={0,0,127}));
    connect(add_T.u2, multivariable_Controller.y[1]) annotation (Line(points={{
            -62,80},{-52,80},{-52,78},{-38,78},{-38,70.5},{-20.6,70.5}}, color=
            {0,0,127}));
    connect(add_P.u2, multivariable_Controller.y[2]) annotation (Line(points={{
            -62,50},{-52,50},{-52,52},{-30,52},{-30,69.5},{-20.6,69.5}}, color=
            {0,0,127}));
    connect(add_T.y, Physical_System.nFan) annotation (Line(points={{-85,74},{
            -92,74},{-92,30},{-15,30},{-15,22}}, color={0,0,127}));
    connect(add_P.y, Physical_System.Ventil) annotation (Line(points={{-85,44},
            {-88,44},{-88,28},{7.8,28},{7.8,22}}, color={0,0,127}));
    connect(step_T.y, add_T.u1) annotation (Line(points={{-121,78},{-88,78},{
            -88,70},{-54,70},{-54,68},{-62,68}}, color={0,0,127}));
    connect(step_P.y, add_P.u1) annotation (Line(points={{-123,42},{-86,42},{
            -50,42},{-50,38},{-62,38}}, color={0,0,127}));
    connect(booleanStep.y, multivariable_Controller.activeInput) annotation (
        Line(points={{19,44},{19,44},{-10,44},{-10,59}}, color={255,0,255}));
    connect(T_R.y, multivariable_Controller.u_s[1]) annotation (Line(points={{
            55,78},{1.77636e-015,78},{1.77636e-015,79}}, color={0,0,127}));
    connect(P_R.y, multivariable_Controller.u_s[2]) annotation (Line(points={{
            55,60},{34,60},{34,70},{8.88178e-016,70},{8.88178e-016,77}}, color=
            {0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Identification;
  end HP_HT_Control;
  annotation (uses(Supermarktmodelle_342(version="1"), Modelica(version="3.2.2")));
end FMU_Physical;

model FMU_Physical_HP_HT_Control_Identification
 extends FMU_Physical.HP_HT_Control.Identification(multivariable_Controller(B={
          {1,0},{0,1}}, D={{1,0},{0,1}}));
  annotation(experiment(
    StopTime=100000,
    __Dymola_NumberOfIntervals=500,
    Tolerance=1e-007,
    __Dymola_Algorithm="dassl"));
end FMU_Physical_HP_HT_Control_Identification;
