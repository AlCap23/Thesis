within ;
package PI_SP
  model PIController_sw
    "PI-Controller with optional inputs for gain scheduling"

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

    parameter Boolean invertFeedback=false "true, if feedback Modelica.SIUnitsgnal is inverted"
      annotation (dialog(group="Settings"));

    parameter Real offset = 0.0 "Operating point, added to proportional output"
      annotation (dialog(enable=(controllerType=="P"), group="Settings"));

    parameter String integralParameterType="Ti"
      "Controller Structure [k*(u + 1/Ti*integral(u))] or [k*u + ki*integral(u)]"
      annotation (choices(choice="Ti", choice="ki"), dialog(enable=(
            (controllerType == "PI") or (controllerType == "I")), group="Settings"));

    parameter Real k=1 "Proportional gain of controller"
      annotation (Dialog(enable=(not use_kInput) and (not (controllerType == "I" and integralParameterType == "ki")),  group="Setting parameters"));

    parameter Modelica.SIunits.Time Ti(min=Modelica.Constants.small) = 0.5
      "Time constant of Integrator block" annotation (Dialog(enable=(not
            use_TiInput and controllerType <> "P" and integralParameterType ==
            "Ti"), group="Setting parameters"));

    parameter Modelica.SIunits.DampingCoefficient ki(min=Modelica.Constants.small) = 0.5
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

    parameter Boolean use_setpointWeighting=false "Available for PI"
      annotation (Dialog(enable=(controllerType == "PI"), tab="Advanced", group="Setpoint weighting"));

    parameter Real b = 0.5
      "Setpoint weight 0 >= b >= 1" annotation (Dialog(enable=(use_setpointWeighting), tab="Advanced", group="Setpoint weighting"));

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

     parameter Modelica.SIunits.Time activationTime=0.0 "Time when controller is switched on"
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
    Real u_setpointWeighting;
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
        if use_setpointWeighting then
          integral = yInitial - getInputs.k_in*u_setpointWeighting;
        else
          integral = yInitial - getInputs.k_in*u;
        end if;
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
      u_setpointWeighting = (u_m - b*u_s);
    else
      u = (u_s - u_m);
      u_setpointWeighting = (b*u_s - u_m);
    end if;

    if controllerType == "P" then
      der(integral) = 0;
      y_unlim = getInputs.k_in*u + offset;
    elseif controllerType == "I" then
      der(integral) = if getActive.y then ki_internal*(u + u_antiWindUp) else 0.0;
      y_unlim = integral;
    else
      der(integral) = if getActive.y then ki_internal*(u + u_antiWindUp) else 0.0;
      if use_setpointWeighting then
        y_unlim = getInputs.k_in*u_setpointWeighting + integral;
      else
        y_unlim = getInputs.k_in*u + integral;
      end if;
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
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
      Documentation(info="<html>
<p>This controller can be used as P- or PI-controller. The controller can be switched on and off during the simulation. The output can be limited and an anti-wind-up function is included. The initialization can be set as steady state: der(y)=0 or with an initial output: y=yInitial. </p>
</html>"));
  end PIController_sw;

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

  package Testers
  extends TIL.Internals.ClassTypes.ExamplePackage;

    model TestPI

      PIController_sw controller(
        yMin=20,
        yMax=60,
        k=5000,
        controllerType="PI",
        yInitial=45,
        Ti=0.5,
        use_activeInput=false,
        initType="initialOutput",
        use_setpointWeighting=false,
        activationTime=10)           annotation (Placement(transformation(
              extent={{26,-24},{14,-36}}, rotation=0)));
      TIL.VLEFluidComponents.Compressors.EffCompressor effCompressor(
        use_mechanicalPort=true,
        displacement=30e-6,
        volumetricEfficiency=0.95,
        isentropicEfficiency=0.84,
        effectiveIsentropicEfficiency=0.83) annotation (Placement(transformation(
            extent={{-8.0,-8.0},{8.0,8.0}},
            rotation=270,
            origin={-10.0,20.0})));
      TIL.VLEFluidComponents.Boundaries.Boundary boundary1(
        boundaryType="p",
        pFixed=2000000,
        TFixed=331.15) annotation (Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=0,
            origin={40.0,20.0})));
      TIL.VLEFluidComponents.Boundaries.Boundary boundaryWithInputs(
        boundaryType="p",
        use_pressureInput=false,
        TFixed=291.15,
        pFixed=1000000)
                       annotation (Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=0,
            origin={-40.0,20.0})));
      TIL.VLEFluidComponents.Sensors.Sensor_m_flow sensor_m_flow(useTimeConstant=
            true, initialSensorValue=0.049)                      annotation (
          Placement(transformation(
            extent={{-4.0,4.0},{4.0,-4.0}},
            rotation=0,
            origin={20.0,20.0})));
      inner TIL.SystemInformationManager sim(redeclare
          TILMedia.VLEFluidTypes.TILMedia_R410APPF vleFluidType1) annotation (
          Placement(transformation(extent={{60,20},{80,40}}, rotation=0)));
      TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundaryWithInputs(
          use_nInput=true) annotation (Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=90,
            origin={-10.0,0.0})));
      Modelica.Blocks.Sources.Pulse pulse(
        offset=0.05,
        amplitude=0.03,
        period=30,
        startTime=0)
        annotation (Placement(transformation(extent={{60,-40},{40,-20}})));
    equation
      connect(boundaryWithInputs.port, effCompressor.portA) annotation (Line(
          points={{-40,20},{-18,20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_m_flow.portA, effCompressor.portB) annotation (Line(
          points={{17,20},{-2,20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_m_flow.portB, boundary1.port) annotation (Line(
          points={{23,20},{40,20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(effCompressor.rotatoryFlange, rotatoryBoundaryWithInputs.rotatoryFlange)
        annotation (Line(
          points={{-10,12},{-10,0}},
          color={135,135,135},
          thickness=0.5,
          smooth=Smooth.None));
      connect(controller.y, rotatoryBoundaryWithInputs.n_in) annotation (Line(
          points={{13.6,-30},{-10,-30},{-10,-4}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sensor_m_flow.sensorValue, controller.u_m) annotation (Line(
          points={{20,17},{20,-24.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pulse.y, controller.u_s) annotation (Line(points={{39,-30},{30,
              -30},{32,-30},{25.6,-30}}, color={0,0,127}));
      annotation (
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-50},{100,50}},
            initialScale=0.1), graphics={
                                    Text(
              extent={{-82,-46},{82,-50}},
              lineColor={0,0,0},
              textString=
                "This model is write protected, it has to be duplicated to allow modifications")}),
        experiment(StopTime=80),
        __Dymola_experimentSetupOutput);
    end TestPI;

    model TestPI_Input

      PIController_sw   controller(
        yMin=20,
        yMax=60,
        k=5000,
        initType="initialOutput",
        controllerType="PI",
        yInitial=45,
        Ti=0.5,
        use_kInput=true,
        use_TiInput=true,
        use_activeInput=true,
        use_y_notActive=true)
                          annotation (Placement(transformation(extent={{26,-4},{14,
                -16}}, rotation=0)));
      TIL.VLEFluidComponents.Compressors.EffCompressor effCompressor(
        use_mechanicalPort=true,
        displacement=30e-6,
        volumetricEfficiency=0.95,
        isentropicEfficiency=0.84,
        effectiveIsentropicEfficiency=0.83) annotation (Placement(transformation(
            extent={{-8.0,-8.0},{8.0,8.0}},
            rotation=270,
            origin={-10.0,20.0})));
      TIL.VLEFluidComponents.Boundaries.Boundary boundary1(
        boundaryType="p",
        pFixed=2000000,
        TFixed=331.15) annotation (Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=0,
            origin={40.0,20.0})));
      TIL.VLEFluidComponents.Boundaries.Boundary boundaryWithInputs(
        boundaryType="p",
        use_pressureInput=true,
        pFixed=1000000,
        TFixed=291.15) annotation (Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=0,
            origin={-40.0,20.0})));
      TIL.VLEFluidComponents.Sensors.Sensor_m_flow sensor_m_flow(useTimeConstant=
            true, initialSensorValue=0.049)                      annotation (
          Placement(transformation(
            extent={{-4.0,4.0},{4.0,-4.0}},
            rotation=0,
            origin={20.0,20.0})));
      inner TIL.SystemInformationManager sim(redeclare
          TILMedia.VLEFluidTypes.TILMedia_R410APPF vleFluidType1) annotation (
          Placement(transformation(extent={{60,20},{80,40}}, rotation=0)));
      TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundaryWithInputs(
          use_nInput=true) annotation (Placement(transformation(
            extent={{-4.0,-10.0},{4.0,10.0}},
            rotation=90,
            origin={-10.0,0.0})));
      Modelica.Blocks.Sources.RealExpression setpoint_m_flow(y=0.05) annotation (
          Placement(transformation(
            origin={60,-10},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Modelica.Blocks.Sources.Trapezoid trapezoid(
        offset=10e5,
        startTime=7.5,
        amplitude=-2e5,
        rising=1e-6,
        width=10,
        falling=1e-6,
        period=20) annotation (Placement(transformation(extent={{-88,16},{-68,36}},
              rotation=0)));
      Modelica.Blocks.Sources.RealExpression k_value(y=5000) annotation (Placement(
            transformation(
            origin={60,-40},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Modelica.Blocks.Sources.RealExpression Ti_value(y=0.5) annotation (Placement(
            transformation(
            origin={60,-26},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Modelica.Blocks.Sources.BooleanPulse booleanPulse(period=10, startTime=20)
        annotation (Placement(transformation(
            extent={{7,-7},{-7,7}},
            rotation=0,
            origin={79,7})));
      Modelica.Blocks.Logical.Not not1 annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=180,
            origin={57,7})));
      Modelica.Blocks.Sources.RealExpression y_notActive(y=30) annotation (
          Placement(transformation(
            origin={0,-26},
            extent={{10,-10},{-10,10}},
            rotation=180)));
    equation
      connect(boundaryWithInputs.port, effCompressor.portA) annotation (Line(
          points={{-40,20},{-18,20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_m_flow.portA, effCompressor.portB) annotation (Line(
          points={{17,20},{-2,20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_m_flow.portB, boundary1.port) annotation (Line(
          points={{23,20},{40,20}},
          color={153,204,0},
          thickness=0.5,
          smooth=Smooth.None));
      connect(sensor_m_flow.sensorValue, controller.u_m) annotation (Line(
          points={{20,17},{20,-4.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(effCompressor.rotatoryFlange, rotatoryBoundaryWithInputs.rotatoryFlange)
        annotation (Line(
          points={{-10,12},{-10,5},{-10,0},{-10,0}},
          color={135,135,135},
          thickness=0.5,
          smooth=Smooth.None));
      connect(controller.y, rotatoryBoundaryWithInputs.n_in) annotation (Line(
          points={{13.6,-10},{-10,-10},{-10,-4}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(setpoint_m_flow.y, controller.u_s) annotation (Line(
          points={{49,-10},{25.6,-10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(trapezoid.y, boundaryWithInputs.p_in) annotation (Line(
          points={{-67,26},{-44,26}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(k_value.y, controller.k_in) annotation (Line(
          points={{49,-40},{40,-40},{40,-14},{25.6,-14}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(Ti_value.y, controller.Ti_in) annotation (Line(
          points={{49,-26},{44,-26},{44,-12},{25.6,-12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(not1.u, booleanPulse.y) annotation (Line(points={{65.4,7},{68.7,7},{
              68.7,7},{71.3,7}}, color={255,0,255}));
      connect(not1.y, controller.activeInput) annotation (Line(points={{49.3,7},{44,
              7},{44,-6.2},{25.6,-6.2}}, color={255,0,255}));
      connect(y_notActive.y, controller.y_notActive)
        annotation (Line(points={{11,-26},{20,-26},{20,-15.8}}, color={0,0,127}));
      annotation (
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-50},{100,50}},
            initialScale=0.1), graphics={
                                    Text(
              extent={{-100,-46},{64,-50}},
              lineColor={0,0,0},
              textString=
                "This model is write protected, it has to be duplicated to allow modifications")}),
        experiment(StopTime=80),
        __Dymola_experimentSetupOutput);
    end TestPI_Input;
    annotation(__Dymola_Protection(
               nestedAllowDuplicate = true,
               nestedShowDiagram=true,
               nestedShowText=true));
  end Testers;
  annotation (uses(
      Modelica(version="3.2.2"),
      TILMedia(version="3.4.2"),
      TIL(version="3.4.1")));
end PI_SP;
