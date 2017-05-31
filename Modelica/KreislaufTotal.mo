




package Kreislauf
  model Julius "Supermarket Refrigeration System CO2"
   parameter Real pHochdruckStart=100e5;

   parameter Real length_aftergascooler=10;
   parameter Real length_aftercompressor=5;
   parameter Real length_aftervalve=5;
   parameter Real length_beforecompressor=10;
   parameter Real innerdiameter=0.1;

    TIL.VLEFluidComponents.Valves.OrificeValve MTValve(effectiveFlowAreaFixed=
          0.2e-5, use_effectiveFlowAreaInput=true) annotation (Placement(
          transformation(
          extent={{-8.0,-4.0},{8.0,4.0}},
          rotation=0,
          origin={-160,-20})));
    TIL.OtherComponents.Thermal.HeatBoundary
      heatBoundaryWithInputs_MTCabinet_Einzeln(
      TFixed(displayUnit="K") = 273.15 + 21,
      boundaryType="Q_flow",
      Q_flowFixed(displayUnit="kW") = -50000,
      use_heatFlowRateInput=false)            annotation (Placement(
          transformation(
          extent={{-4.0,-6.0},{4.0,6.0}},
          rotation=0,
          origin={-34,-34})));
    TIL.OtherComponents.Controllers.PIController PI_MTValve(
      yMin=0.2e-12,
      yMax=0.1e-3,
      controllerType="PI",
      initType="initialOutput",
      invertFeedback=true,
      yInitial=0.68e-5,
      activationTime=500,
      Ti=200,
      k=1e-7,
      use_activeInput=false)
                 annotation (Placement(transformation(
          origin={-144,12},
          extent={{-6,6},{6,-6}},
          rotation=180)));
    Modelica.Blocks.Sources.RealExpression setPoint_MT_Valve(y=8) annotation (
        Placement(transformation(
          origin={-116,12},
          extent={{10,-10},{-10,10}},
          rotation=0)));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_5(
      vleFluidType=sim.vleFluidType1,
      pressureStateID=5,
      pInitial(displayUnit="Pa") = 28.004e5,
      fixedInitialPressure=false)            annotation (Placement(transformation(
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
      annotation (Placement(transformation(extent={{72,256},{92,276}},   rotation=
             0)));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Components.Compressor.SimplifiedRack compressor(
      displacement=0.000204204,
      nFixed=50,
      use_mechanicalPort=true,
      C=0.9,
      K=0.1) annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=0,
          origin={80,156})));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_2(
      vleFluidType=sim.vleFluidType1,
      pressureStateID=2,
      fixedInitialPressure=true,
      pInitial(displayUnit="Pa") = pHochdruckStart)
                                         annotation (Placement(transformation(
          extent={{-6.0,-6.0},{6.0,6.0}},
          rotation=0,
          origin={48,202})));
    TIL.VLEFluidComponents.JunctionElements.VolumeJunction junction_from_IHX(
      volume=1e-2,
      hInitial=451e3,
      pressureStateID=5)
                      annotation (Placement(transformation(
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
      initialSensorValue=pressureState_2.pInitial)
                                  annotation (Placement(transformation(
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
    TIL.VLEFluidComponents.Valves.OrificeValve RecValve(effectiveFlowAreaFixed=
          0.9e-5, use_effectiveFlowAreaInput=true) annotation (Placement(
          transformation(
          extent={{-8.0,-4.0},{8.0,4.0}},
          rotation=0,
          origin={-120,62})));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_3(
        pInitial(displayUnit="Pa") = 34.9e5, pressureStateID=3) annotation (
        Placement(transformation(
          extent={{-6.0,-6.0},{6.0,6.0}},
          rotation=90,
          origin={-258,118})));
    TIL.VLEFluidComponents.Sensors.Sensor_p sensor_MP(useTimeConstant=true,
        initialSensorValue=3490000) annotation (Placement(transformation(
          extent={{-4.0,-4.0},{4.0,4.0}},
          rotation=0,
          origin={-170,84})));
    TIL.OtherComponents.Controllers.PIController PI_RecValve(
      yMin=1e-10,
      yMax=1e-1,
      k=1e-8,
      Ti=10,
      controllerType="PI",
      initType="initialOutput",
      yInitial=1.02e-5,
      invertFeedback=true,
      use_activeInput=true)
                           annotation (Placement(transformation(extent={{-176,100},
              {-164,112}}, rotation=0)));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint5(stateViewerIndex=5)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=90,
          origin={-270,98})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint2(stateViewerIndex=2)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=90,
          origin={66,144})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint3(stateViewerIndex=3)
      annotation (Placement(transformation(extent={{26,206},{34,214}})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint10(stateViewerIndex=
          10)
      annotation (Placement(transformation(extent={{-144,72},{-136,80}})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint7(stateViewerIndex=7)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=0,
          origin={-130,-8})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint1(stateViewerIndex=1)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=90,
          origin={68,42})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint6(stateViewerIndex=6)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=270,
          origin={-190,10})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint9(stateViewerIndex=9)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=0,
          origin={-80,76})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint4(stateViewerIndex=4)
      annotation (Placement(transformation(extent={{-262,206},{-254,214}})));
    TIL.VLEFluidComponents.Sensors.Sensor_T sensor_T(useTimeConstant=false,
        initialSensorValue=283.15)
      annotation (Placement(transformation(extent={{-196,210},{-188,218}})));
    TIL.OtherComponents.Controllers.PIController PI_HPComp(
      controllerType="PI",
      initType="initialOutput",
      invertFeedback=true,
      Ti=10,
      k=70e-4,
      yMax=350,
      yMin=20,
      yInitial=100,
      use_activeInput=false)
              annotation (Placement(transformation(extent={{122,152},{110,164}},
            rotation=0)));
    Modelica.Blocks.Sources.RealExpression setPoint_HP_Comp(y=28e5) annotation (
        Placement(transformation(
          origin={140,158},
          extent={{10,-10},{-10,10}},
          rotation=0)));
    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundaryWithInputs(
        use_nInput=true, nFixed=40) annotation (Placement(transformation(
          extent={{-4.0,-10.0},{4.0,10.0}},
          rotation=180,
          origin={100,156})));
    TIL.VLEFluidComponents.Sensors.Sensor_p sensor_MP2(useTimeConstant=true,
      tau=5,
      initialSensorValue=2800000)   annotation (Placement(transformation(
          extent={{-4.0,-4.0},{4.0,4.0}},
          rotation=270,
          origin={88,144})));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
      Fan1(
      use_mechanicalPort=true,
      V_flow_Start=15,
      V_flow_nominal=4,
      n_nominal=11.5,
      dp_nominal=60,
      V_flow0=6,
      deltaV_flow=1,
      eta=0.35) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={-76,248})));
    TIL.GasComponents.Boundaries.Boundary boundary(boundaryType="p",
      use_temperatureInput=true,
      TFixed=303.15)
      annotation (Placement(transformation(extent={{-102,238},{-94,258}})));
    TIL.GasComponents.HydraulicResistors.HydraulicResistor hydraulicResistor(
        zeta_fixed=0.7, hydraulicDiameter=0.8)
      annotation (Placement(transformation(extent={{-60,246},{-48,250}})));
    TIL.GasComponents.Boundaries.Boundary boundary1(boundaryType="p")
      annotation (Placement(transformation(extent={{-4,-10},{4,10}},
          rotation=90,
          origin={-22,222})));
    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary3(
                                                                     use_nInput=
         true)
      annotation (Placement(transformation(extent={{-94,250},{-86,270}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(y_start=10, T=20)
      annotation (Placement(transformation(extent={{-120,256},{-112,264}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder1(
      y_start=35e5,
      initType=Modelica.Blocks.Types.Init.InitialOutput,
      T=50)
      annotation (Placement(transformation(extent={{-206,96},{-186,116}})));
    TIL.HeatExchangers.FinAndTube.GasVLEFluid.CrossFlowHX Gascooler(
      gasCellFlowType="flow B-A",
      initVLEFluid="linearEnthalpyDistribution",
      redeclare model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum,
      m_flowVLEFluidStart=0.78,
      redeclare model WallMaterial = TILMedia.SolidTypes.TILMedia_Copper,
      redeclare model FinSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.Haaf,
      redeclare
        TIL3_AddOn_Supermarkt_Wurm_Februar17.Components.Gascooler.Geometry.Kl_Sup_WRG_84_119EC28V
        hxGeometry,
      redeclare model FinEfficiencyModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.ConstFinEfficiency,
      pressureStateID=2,
      hInitialVLEFluid_Cell1=260e3,
      hInitialVLEFluid_CellN=500e3,
      pVLEFluidStart=pressureState_2.pInitial,
      redeclare model TubeSidePressureDropModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop,
      cellOrientation="B",
      redeclare model WallHeatConductionModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.ConstantR
          (constantR=1e-8),
      redeclare model TubeSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ShahChenGnielinskiDittusBoelter
          (alpha_initial=5e3),
      TInitialWall(displayUnit="K") = 340,
      nCells=10)
      annotation (Placement(transformation(extent={{-36,188},{-8,216}})));

    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary1(
                                                                     use_nInput=
         true)
      annotation (Placement(transformation(extent={{-94,220},{-86,240}})));
    TIL.GasComponents.HydraulicResistors.HydraulicResistor hydraulicResistor1(
        zeta_fixed=0.7, hydraulicDiameter=0.8)
      annotation (Placement(transformation(extent={{-62,214},{-50,218}})));
    TIL.GasComponents.Boundaries.Boundary boundary2(
                                                   boundaryType="p",
      use_temperatureInput=true,
      TFixed=303.15)
      annotation (Placement(transformation(extent={{-102,206},{-94,226}})));
    TIL.GasComponents.JunctionElements.VolumeJunction junction2(volume=1e-3,
        fixedInitialPressure=false)
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},
          rotation=90,
          origin={-40,216})));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
      Fan2(
      use_mechanicalPort=true,
      V_flow_Start=15,
      V_flow_nominal=4,
      n_nominal=11.5,
      dp_nominal=60,
      V_flow0=6,
      deltaV_flow=1,
      eta=0.35) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={-76,216})));
    TIL.VLEFluidComponents.Sensors.Sensor_superheating sensor_MT(
      useTimeConstant=true,
      initialSensorValue=8,
      tau=1)
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},
          rotation=90,
          origin={66,94})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint8(stateViewerIndex=8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=0,
          origin={2,-6})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint11(stateViewerIndex=
          11)
      annotation (Placement(transformation(extent={{-228,70},{-220,78}})));
    TIL.VLEFluidComponents.Tubes.Tube tube(
      pressureStateID=5,
      redeclare TIL.VLEFluidComponents.Tubes.Geometry.Tube6x1 tubeGeometry,
      enableHeatPorts=true,
      nCells=3,
      redeclare model TubeSideHeatTransferModel =
          TIL.VLEFluidComponents.Tubes.TransportPhenomena.HeatTransfer.ConstantAlpha
          (constantAlpha=5000),
      falseDynamicsTimeConstant=2,
      initVLEFluid="constantTemperature",
      pressureDropInitialVLEFluid=0,
      fixedPressureDropInitialVLEFluid=false,
      TInitialVLEFluid=268.15,
      TInitialWall=268.15,
      pStart=2800000)
      annotation (Placement(transformation(extent={{-24,-18},{-8,-22}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=35e5)
      annotation (Placement(transformation(extent={{-242,96},{-222,116}})));
    Modelica.Blocks.Interfaces.RealInput unFan
      annotation (Placement(transformation(extent={{-338,260},{-298,300}}),
          iconTransformation(extent={{-338,260},{-298,300}})));
    Modelica.Blocks.Interfaces.RealInput uVentil
      annotation (Placement(transformation(extent={{-340,-20},{-300,20}}),
          iconTransformation(extent={{-340,-20},{-300,20}})));
    Modelica.Blocks.Interfaces.RealInput uTUmgebung
      annotation (Placement(transformation(extent={{-340,124},{-300,164}}),
          iconTransformation(extent={{-340,124},{-300,164}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder3(
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=300,
      T=100)
      annotation (Placement(transformation(extent={{-162,246},{-154,254}})));
    TIL.VLEFluidComponents.Tubes.Tube tube1(
      falseDynamicsTimeConstant=2,
      initVLEFluid="constantTemperature",
      pressureDropInitialVLEFluid=0,
      fixedPressureDropInitialVLEFluid=false,
      redeclare TIL.VLEFluidComponents.Tubes.Geometry.TubeGeometry tubeGeometry(
        innerDiameter=innerdiameter,
        wallThickness=0.01,
        length=length_aftergascooler),
      enableHeatPorts=false,
      redeclare model TubeSideHeatTransferModel =
          TIL.VLEFluidComponents.Tubes.TransportPhenomena.HeatTransfer.ConstantAlpha
          (constantAlpha=5000),
      pStart=pHochdruckStart,
      nCells=10,
      pressureStateID=2,
      TInitialVLEFluid(displayUnit="K") = 300,
      TInitialWall(displayUnit="K") = 300)
      annotation (Placement(transformation(extent={{-154,204},{-170,200}})));
    TIL.VLEFluidComponents.Tubes.Tube tube2(
      falseDynamicsTimeConstant=2,
      initVLEFluid="constantTemperature",
      pressureDropInitialVLEFluid=0,
      fixedPressureDropInitialVLEFluid=false,
      enableHeatPorts=false,
      redeclare model TubeSideHeatTransferModel =
          TIL.VLEFluidComponents.Tubes.TransportPhenomena.HeatTransfer.ConstantAlpha
          (constantAlpha=5000),
      pStart=pHochdruckStart,
      nCells=10,
      pressureStateID=2,
      TInitialVLEFluid(displayUnit="K") = 350,
      TInitialWall(displayUnit="K") = 350,
      redeclare TIL.VLEFluidComponents.Tubes.Geometry.TubeGeometry tubeGeometry(
        innerDiameter=innerdiameter,
        wallThickness=0.01,
        length=length_aftercompressor))
      annotation (Placement(transformation(extent={{80,204},{64,200}})));
    TIL.VLEFluidComponents.Tubes.Tube tube3(
      falseDynamicsTimeConstant=2,
      initVLEFluid="constantTemperature",
      pressureDropInitialVLEFluid=0,
      fixedPressureDropInitialVLEFluid=false,
      enableHeatPorts=false,
      redeclare model TubeSideHeatTransferModel =
          TIL.VLEFluidComponents.Tubes.TransportPhenomena.HeatTransfer.ConstantAlpha
          (constantAlpha=5000),
      nCells=10,
      pressureStateID=3,
      TInitialVLEFluid(displayUnit="K") = 270,
      TInitialWall(displayUnit="K") = 270,
      redeclare TIL.VLEFluidComponents.Tubes.Geometry.TubeGeometry tubeGeometry(
        innerDiameter=innerdiameter,
        wallThickness=0.01,
        length=length_aftervalve),
      pStart=3500000)
      annotation (Placement(transformation(extent={{-254,64},{-238,60}})));
    Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=true)
      annotation (Placement(transformation(extent={{-206,146},{-186,166}})));
  equation

    connect(MTValve.portB,pressureState_5. portB) annotation (Line(
        points={{-152,-20},{-124,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_HP.port,HPValve. portA) annotation (Line(
        points={{-242,178},{-258,178},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(MPReciever.portGas,RecValve. portA) annotation (Line(
        points={{-195,62},{-128,62}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(HPValve.portB,pressureState_3. portA) annotation (Line(
        points={{-258,140},{-258,124}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_MP.port,MPReciever. portGas) annotation (Line(
        points={{-170,80},{-170,62},{-195,62}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint1.sensorPort, junction_from_IHX.portA) annotation (Line(
        points={{72,42},{78,42},{78,58}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint10.sensorPort, RecValve.portA) annotation (Line(
        points={{-140,72},{-140,62},{-128,62}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint7.sensorPort, MTValve.portB) annotation (Line(
        points={{-130,-12},{-130,-20},{-152,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint4.sensorPort,HPValve. portA) annotation (Line(
        points={{-258,206},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_MP.sensorValue,PI_RecValve. u_m) annotation (Line(
        points={{-170,86.2},{-170,100.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(setPoint_MT_Valve.y,PI_MTValve. u_s) annotation (Line(
        points={{-127,12},{-138.4,12}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(PI_RecValve.y,RecValve. effectiveFlowArea_in) annotation (Line(
        points={{-163.6,106},{-120,106},{-120,67}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(PI_MTValve.y,MTValve. effectiveFlowArea_in) annotation (Line(
        points={{-150.4,12},{-160,12},{-160,-15}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(statePoint2.sensorPort, compressor.portA) annotation (Line(
        points={{70,144},{80,144},{80,148}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(rotatoryBoundaryWithInputs.n_in,PI_HPComp. y) annotation (Line(
        points={{104,156},{106,156},{106,158},{109.6,158}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(PI_HPComp.u_s,setPoint_HP_Comp. y) annotation (Line(
        points={{121.6,158},{129,158}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(sensor_MP2.port,compressor. portA) annotation (Line(
        points={{84,144},{80,144},{80,148}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_MP2.sensorValue,PI_HPComp. u_m) annotation (Line(
        points={{90.2,144},{116,144},{116,152.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(compressor.rotatoryFlange,rotatoryBoundaryWithInputs. rotatoryFlange)
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
    connect(statePoint3.sensorPort,pressureState_2. portB) annotation (Line(
        points={{30,206},{30,202},{42,202}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_T.port,HPValve. portA) annotation (Line(
        points={{-192,210},{-192,202},{-258,202},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(firstOrder1.y,PI_RecValve. u_s) annotation (Line(
        points={{-185,106},{-175.6,106}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(MPReciever.portLiquid, MTValve.portA) annotation (Line(
        points={{-200,48},{-200,-20},{-168,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pressureState_2.portB, Gascooler.portB_vle) annotation (Line(
        points={{42,202},{-8,202}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Gascooler.portA_gas, boundary1.port) annotation (Line(
        points={{-22,216},{-22,222}},
        color={255,153,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Fan1.rotatoryFlange, rotatoryBoundary3.rotatoryFlange) annotation (
        Line(
        points={{-76,254},{-76,260},{-90,260}},
        color={135,135,135},
        thickness=0.5,
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
    connect(Fan2.rotatoryFlange, rotatoryBoundary1.rotatoryFlange) annotation (
        Line(
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
        points={{-40,212},{-40,180},{-22,180},{-22,188}},
        color={255,153,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(rotatoryBoundary3.n_in, firstOrder.y) annotation (Line(
        points={{-94,260},{-111.6,260}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(rotatoryBoundary1.n_in, firstOrder.y) annotation (Line(
        points={{-94,230},{-104,230},{-104,260},{-111.6,260}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(sensor_MT.sensorValue, PI_MTValve.u_m) annotation (Line(
        points={{63.8,94},{63.8,6.2},{-144,6.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(statePoint11.sensorPort, MPReciever.portInlet) annotation (Line(
        points={{-224,70},{-224,62},{-205,62}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(RecValve.portB, junction_from_IHX.portB) annotation (Line(
        points={{-112,62},{74,62}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint9.sensorPort, junction_from_IHX.portB) annotation (Line(
        points={{-80,72},{-80,62},{74,62}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pressureState_5.portA, tube.portA) annotation (Line(
        points={{-112,-20},{-24,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(tube.portB, junction_from_IHX.portA) annotation (Line(
        points={{-8,-20},{78,-20},{78,58}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint8.sensorPort, junction_from_IHX.portA) annotation (Line(
        points={{2,-10},{2,-20},{78,-20},{78,58}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(heatBoundaryWithInputs_MTCabinet_Einzeln.heatPort, tube.heatPort[
      1]) annotation (Line(
        points={{-34,-34},{-16,-34},{-16,-21.3333}},
        color={204,0,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(firstOrder.u, unFan) annotation (Line(
        points={{-120.8,260},{-224,260},{-224,280},{-318,280}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HPValve.effectiveFlowArea_in, uVentil) annotation (Line(
        points={{-263,148},{-292,148},{-292,0},{-320,0}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(uTUmgebung, firstOrder3.u) annotation (Line(points={{-320,144},{
            -320,250},{-162.8,250}},  color={0,0,127}));
    connect(firstOrder3.y, boundary.T_in) annotation (Line(points={{-153.6,250},
            {-118,250},{-118,242},{-102,242}},
                                         color={0,0,127}));
    connect(firstOrder3.y, boundary2.T_in) annotation (Line(points={{-153.6,250},
            {-118,250},{-118,210},{-102,210}},color={0,0,127}));
    connect(tube1.portA, Gascooler.portA_vle) annotation (Line(
        points={{-154,202},{-96,202},{-36,202}},
        color={153,204,0},
        thickness=0.5));
    connect(tube1.portB, HPValve.portA) annotation (Line(
        points={{-170,202},{-194,202},{-258,202},{-258,156}},
        color={153,204,0},
        thickness=0.5));
    connect(pressureState_2.portA, tube2.portB) annotation (Line(
        points={{54,202},{64,202}},
        color={153,204,0},
        thickness=0.5));
    connect(compressor.portB, tube2.portA) annotation (Line(
        points={{80,164},{80,164},{80,202}},
        color={153,204,0},
        thickness=0.5));
    connect(tube3.portB, MPReciever.portInlet) annotation (Line(
        points={{-238,62},{-205,62}},
        color={153,204,0},
        thickness=0.5));
    connect(tube3.portA, pressureState_3.portB) annotation (Line(
        points={{-254,62},{-258,62},{-258,112}},
        color={153,204,0},
        thickness=0.5));
    connect(statePoint5.sensorPort, pressureState_3.portB) annotation (Line(
        points={{-266,98},{-258,98},{-258,112}},
        color={153,204,0},
        thickness=0.5));
    connect(realExpression.y, firstOrder1.u) annotation (Line(points={{-221,
            106},{-208,106}},      color={0,0,127}));
    connect(junction_from_IHX.portC, compressor.portA) annotation (Line(
        points={{78,66},{80,66},{80,148}},
        color={153,204,0},
        thickness=0.5));
    connect(sensor_MT.port, compressor.portA) annotation (Line(
        points={{70,94},{80,94},{80,148}},
        color={153,204,0},
        thickness=0.5));
    connect(booleanStep.y, PI_RecValve.activeInput) annotation (Line(points={{
            -185,156},{-176,156},{-176,154},{-176,102.2},{-175.6,102.2}}, color=
           {255,0,255}));
    annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-340,-60},{160,300}},
          initialScale=0.1)),
      experiment(StopTime=1.3e5, __Dymola_Algorithm="Dassl"),
      __Dymola_experimentSetupOutput(equdistant=false),
      Icon(coordinateSystem(extent={{-340,-60},{160,300}},
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
  end Julius;

  model Julius_Tester
    Kreislauf.Julius julius
      annotation (Placement(transformation(extent={{-28,-2},{22,34}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=10)
      annotation (Placement(transformation(extent={{-68,22},{-48,42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15 + 25)
      annotation (Placement(transformation(extent={{-68,8},{-48,28}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=4e-6)
      annotation (Placement(transformation(extent={{-68,-6},{-48,14}})));
  equation
    connect(julius.unFan, realExpression.y)
      annotation (Line(points={{-25.8,32},{-47,32}}, color={0,0,127}));
    connect(julius.uTUmgebung, realExpression1.y) annotation (Line(points={{-26,
            18.4},{-37,18.4},{-37,18},{-47,18}}, color={0,0,127}));
    connect(julius.uVentil, realExpression2.y)
      annotation (Line(points={{-26,4},{-47,4}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}})));
  end Julius_Tester;

  model Fan_Pressure1 "Supermarket Refrigeration System CO2"
   parameter Real pHochdruckStart=100e5;

   parameter Real length_aftergascooler=10;
   parameter Real length_aftercompressor=5;
   parameter Real length_aftervalve=5;
   parameter Real length_beforecompressor=10;
   parameter Real innerdiameter=0.1;

    inner TIL.SystemInformationManager sim(
      redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType1,
      redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType2,
      redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType3,
      redeclare TILMedia.GasTypes.TILMedia_MoistAir gasType1,
      redeclare TILMedia.GasTypes.TILMedia_MoistAir gasType2,
      redeclare TILMedia.LiquidTypes.TILMedia_Glysantin_60 liquidType1)
      annotation (Placement(transformation(extent={{72,256},{92,276}},   rotation=
             0)));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_2(
      vleFluidType=sim.vleFluidType1,
      pressureStateID=2,
      fixedInitialPressure=true,
      pInitial(displayUnit="Pa") = pHochdruckStart)
                                         annotation (Placement(transformation(
          extent={{-6.0,-6.0},{6.0,6.0}},
          rotation=0,
          origin={48,202})));
    TIL.VLEFluidComponents.Valves.OrificeValve HPValve(
      vleFluidType=sim.vleFluidType1,
      x(start=0.5),
      effectiveFlowAreaFixed=2e-6,
      use_effectiveFlowAreaInput=true,
      mdotSmooth=0.1e-1,
      effectiveFlowAreaTypical=4e-6,
      portB(p(start=35e5)))            annotation (Placement(transformation(
          extent={{-8,4},{8,-4}},
          rotation=270,
          origin={-258,148})));
    TIL.VLEFluidComponents.Sensors.Sensor_p sensor_HP(
      useTimeConstant=true,
      tau=5,
      initialSensorValue=pressureState_2.pInitial)
                                  annotation (Placement(transformation(
          extent={{-4.0,-4.0},{4.0,4.0}},
          rotation=270,
          origin={-238,178})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint3(stateViewerIndex=3)
      annotation (Placement(transformation(extent={{26,206},{34,214}})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint4(stateViewerIndex=4)
      annotation (Placement(transformation(extent={{-262,206},{-254,214}})));
    TIL.VLEFluidComponents.Sensors.Sensor_T sensor_T(useTimeConstant=false,
        initialSensorValue=283.15)
      annotation (Placement(transformation(extent={{-196,210},{-188,218}})));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
      Fan1(
      use_mechanicalPort=true,
      V_flow_Start=15,
      V_flow_nominal=4,
      n_nominal=11.5,
      dp_nominal=60,
      V_flow0=6,
      deltaV_flow=1,
      eta=0.35) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={-76,248})));
    TIL.GasComponents.Boundaries.Boundary boundary(boundaryType="p",
      use_temperatureInput=true,
      TFixed=303.15)
      annotation (Placement(transformation(extent={{-102,238},{-94,258}})));
    TIL.GasComponents.HydraulicResistors.HydraulicResistor hydraulicResistor(
        zeta_fixed=0.7, hydraulicDiameter=0.8)
      annotation (Placement(transformation(extent={{-60,246},{-48,250}})));
    TIL.GasComponents.Boundaries.Boundary boundary1(boundaryType="p")
      annotation (Placement(transformation(extent={{-4,-10},{4,10}},
          rotation=90,
          origin={-22,222})));
    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary3(
                                                                     use_nInput=
         true)
      annotation (Placement(transformation(extent={{-94,250},{-86,270}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(y_start=10, T=20)
      annotation (Placement(transformation(extent={{-120,256},{-112,264}})));
    TIL.HeatExchangers.FinAndTube.GasVLEFluid.CrossFlowHX Gascooler(
      gasCellFlowType="flow B-A",
      initVLEFluid="linearEnthalpyDistribution",
      redeclare model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum,
      m_flowVLEFluidStart=0.78,
      redeclare model WallMaterial = TILMedia.SolidTypes.TILMedia_Copper,
      redeclare model FinSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.Haaf,
      redeclare
        TIL3_AddOn_Supermarkt_Wurm_Februar17.Components.Gascooler.Geometry.Kl_Sup_WRG_84_119EC28V
        hxGeometry,
      redeclare model FinEfficiencyModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.ConstFinEfficiency,
      pressureStateID=2,
      hInitialVLEFluid_Cell1=260e3,
      hInitialVLEFluid_CellN=500e3,
      pVLEFluidStart=pressureState_2.pInitial,
      redeclare model TubeSidePressureDropModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop,
      cellOrientation="B",
      redeclare model WallHeatConductionModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.ConstantR
          (constantR=1e-8),
      redeclare model TubeSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ShahChenGnielinskiDittusBoelter
          (alpha_initial=5e3),
      TInitialWall(displayUnit="K") = 340,
      nCells=10)
      annotation (Placement(transformation(extent={{-36,188},{-8,216}})));

    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary1(
                                                                     use_nInput=
         true)
      annotation (Placement(transformation(extent={{-94,220},{-86,240}})));
    TIL.GasComponents.HydraulicResistors.HydraulicResistor hydraulicResistor1(
        zeta_fixed=0.7, hydraulicDiameter=0.8)
      annotation (Placement(transformation(extent={{-62,214},{-50,218}})));
    TIL.GasComponents.Boundaries.Boundary boundary2(
                                                   boundaryType="p",
      use_temperatureInput=true,
      TFixed=303.15)
      annotation (Placement(transformation(extent={{-102,206},{-94,226}})));
    TIL.GasComponents.JunctionElements.VolumeJunction junction2(volume=1e-3,
        fixedInitialPressure=false)
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},
          rotation=90,
          origin={-40,216})));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
      Fan2(
      use_mechanicalPort=true,
      V_flow_Start=15,
      V_flow_nominal=4,
      n_nominal=11.5,
      dp_nominal=60,
      V_flow0=6,
      deltaV_flow=1,
      eta=0.35) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={-76,216})));
    Modelica.Blocks.Interfaces.RealInput unFan
      annotation (Placement(transformation(extent={{-338,260},{-298,300}}),
          iconTransformation(extent={{-338,260},{-298,300}})));
    Modelica.Blocks.Interfaces.RealInput uVentil
      annotation (Placement(transformation(extent={{-342,130},{-302,170}}),
          iconTransformation(extent={{-342,130},{-302,170}})));
    Modelica.Blocks.Interfaces.RealInput uTUmgebung
      annotation (Placement(transformation(extent={{-336,230},{-296,270}}),
          iconTransformation(extent={{-336,230},{-296,270}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder3(
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=300,
      T=100)
      annotation (Placement(transformation(extent={{-162,246},{-154,254}})));
    TIL.VLEFluidComponents.Boundaries.BoundaryOverdetermined
      boundaryOverdetermined(
      m_flowFixed=-0.3,
      hFixed=520e3,
      pFixed(displayUnit="Pa") = 77e5)
      annotation (Placement(transformation(extent={{92,192},{100,212}})));
    TIL.VLEFluidComponents.Boundaries.BoundaryUnderdetermined
      boundaryUnderdetermined annotation (Placement(transformation(
          extent={{-4,-10},{4,10}},
          rotation=90,
          origin={-258,94})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint1(stateViewerIndex=5)
      annotation (Placement(transformation(extent={{-242,124},{-234,132}})));
  equation

    connect(sensor_HP.port,HPValve. portA) annotation (Line(
        points={{-242,178},{-258,178},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint4.sensorPort,HPValve. portA) annotation (Line(
        points={{-258,206},{-258,156}},
        color={153,204,0},
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
    connect(statePoint3.sensorPort,pressureState_2. portB) annotation (Line(
        points={{30,206},{30,202},{42,202}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_T.port,HPValve. portA) annotation (Line(
        points={{-192,210},{-192,202},{-258,202},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pressureState_2.portB, Gascooler.portB_vle) annotation (Line(
        points={{42,202},{-8,202}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Gascooler.portA_gas, boundary1.port) annotation (Line(
        points={{-22,216},{-22,222}},
        color={255,153,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Fan1.rotatoryFlange, rotatoryBoundary3.rotatoryFlange) annotation (
        Line(
        points={{-76,254},{-76,260},{-90,260}},
        color={135,135,135},
        thickness=0.5,
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
    connect(Fan2.rotatoryFlange, rotatoryBoundary1.rotatoryFlange) annotation (
        Line(
        points={{-76,222},{-76,230},{-90,230}},
        color={135,135,135},
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
        points={{-40,212},{-40,180},{-22,180},{-22,188}},
        color={255,153,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(rotatoryBoundary3.n_in, firstOrder.y) annotation (Line(
        points={{-94,260},{-111.6,260}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(rotatoryBoundary1.n_in, firstOrder.y) annotation (Line(
        points={{-94,230},{-104,230},{-104,260},{-111.6,260}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(firstOrder.u, unFan) annotation (Line(
        points={{-120.8,260},{-224,260},{-224,280},{-318,280}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HPValve.effectiveFlowArea_in, uVentil) annotation (Line(
        points={{-263,148},{-292,148},{-292,150},{-322,150}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(uTUmgebung, firstOrder3.u) annotation (Line(points={{-316,250},{
            -316,250},{-162.8,250}},  color={0,0,127}));
    connect(firstOrder3.y, boundary.T_in) annotation (Line(points={{-153.6,250},
            {-118,250},{-118,242},{-102,242}},
                                         color={0,0,127}));
    connect(firstOrder3.y, boundary2.T_in) annotation (Line(points={{-153.6,250},
            {-118,250},{-118,210},{-102,210}},color={0,0,127}));
    connect(pressureState_2.portA, boundaryOverdetermined.port) annotation (
        Line(
        points={{54,202},{76,202},{96,202}},
        color={153,204,0},
        thickness=0.5));
    connect(Gascooler.portA_vle, HPValve.portA) annotation (Line(
        points={{-36,202},{-258,202},{-258,156}},
        color={153,204,0},
        thickness=0.5));
    connect(boundaryUnderdetermined.port, HPValve.portB) annotation (Line(
        points={{-258,94},{-258,140}},
        color={153,204,0},
        thickness=0.5));
    connect(statePoint1.sensorPort, HPValve.portB) annotation (Line(
        points={{-238,124},{-248,124},{-248,120},{-258,120},{-258,140}},
        color={153,204,0},
        thickness=0.5));
    annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-340,-60},{160,300}},
          initialScale=0.1)),
      experiment(StopTime=1.3e5, __Dymola_Algorithm="Dassl"),
      __Dymola_experimentSetupOutput(equdistant=false),
      Icon(coordinateSystem(extent={{-340,-60},{160,300}},
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
  end Fan_Pressure1;

  model Fan_Pressure1_Tester
    Kreislauf.Fan_Pressure1 julius(
      pHochdruckStart=77e5,
      HPValve(effectiveFlowAreaTypical=4e-6),
      boundaryUnderdetermined(port(m_flow(start=0.3))))
      annotation (Placement(transformation(extent={{-28,-2},{22,34}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15 + 25)
      annotation (Placement(transformation(extent={{-68,8},{-48,28}})));
    Modelica.Blocks.Sources.Ramp ramp(
      offset=1e-4,
      height=-1e-4 + 4e-6,
      duration=8,
      startTime=2)
      annotation (Placement(transformation(extent={{-70,-16},{-50,4}})));
    Modelica.Blocks.Sources.Step step(
      height=10,
      offset=10,
      startTime=100)
      annotation (Placement(transformation(extent={{-70,32},{-50,52}})));
  equation
    connect(julius.uTUmgebung, realExpression1.y) annotation (Line(points={{
            -25.6,29},{-37,29},{-37,18},{-47,18}}, color={0,0,127}));
    connect(ramp.y, julius.uVentil) annotation (Line(points={{-49,-6},{-38,-6},
            {-38,19},{-26.2,19}}, color={0,0,127}));
    connect(step.y, julius.unFan) annotation (Line(points={{-49,42},{-42,42},{
            -42,38},{-25.8,38},{-25.8,32}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}}), graphics={Text(
            extent={{-26,56},{28,34}},
            lineColor={28,108,200},
            textString=
                "Bernoulli-Gleichung im Ventil --> da geht die Dichte ein")}));
  end Fan_Pressure1_Tester;

  model Fan_Pressure2 "Supermarket Refrigeration System CO2"
   parameter Real pHochdruckStart=100e5;

   parameter Real length_aftergascooler=10;
   parameter Real length_aftercompressor=5;
   parameter Real length_aftervalve=5;
   parameter Real length_beforecompressor=10;
   parameter Real innerdiameter=0.1;

    inner TIL.SystemInformationManager sim(
      redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType1,
      redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType2,
      redeclare TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType3,
      redeclare TILMedia.GasTypes.TILMedia_MoistAir gasType1,
      redeclare TILMedia.GasTypes.TILMedia_MoistAir gasType2,
      redeclare TILMedia.LiquidTypes.TILMedia_Glysantin_60 liquidType1)
      annotation (Placement(transformation(extent={{72,256},{92,276}},   rotation=
             0)));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_2(
      vleFluidType=sim.vleFluidType1,
      pressureStateID=2,
      fixedInitialPressure=true,
      pInitial(displayUnit="Pa") = pHochdruckStart)
                                         annotation (Placement(transformation(
          extent={{-6.0,-6.0},{6.0,6.0}},
          rotation=0,
          origin={48,202})));
    TIL.VLEFluidComponents.Valves.OrificeValve HPValve(
      vleFluidType=sim.vleFluidType1,
      x(start=0.5),
      effectiveFlowAreaFixed=2e-6,
      use_effectiveFlowAreaInput=true,
      mdotSmooth=0.1e-1,
      effectiveFlowAreaTypical=4e-6,
      portB(p(start=35e5)))            annotation (Placement(transformation(
          extent={{-8,4},{8,-4}},
          rotation=270,
          origin={-258,148})));
    TIL.VLEFluidComponents.Sensors.Sensor_p sensor_HP(
      useTimeConstant=true,
      tau=5,
      initialSensorValue=pressureState_2.pInitial)
                                  annotation (Placement(transformation(
          extent={{-4.0,-4.0},{4.0,4.0}},
          rotation=270,
          origin={-238,178})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint3(stateViewerIndex=3)
      annotation (Placement(transformation(extent={{26,206},{34,214}})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint4(stateViewerIndex=4)
      annotation (Placement(transformation(extent={{-262,206},{-254,214}})));
    TIL.VLEFluidComponents.Sensors.Sensor_T sensor_T(useTimeConstant=false,
        initialSensorValue=283.15)
      annotation (Placement(transformation(extent={{-196,210},{-188,218}})));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
      Fan1(
      use_mechanicalPort=true,
      V_flow_Start=15,
      V_flow_nominal=4,
      n_nominal=11.5,
      dp_nominal=60,
      V_flow0=6,
      deltaV_flow=1,
      eta=0.35) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={-76,248})));
    TIL.GasComponents.Boundaries.Boundary boundary(boundaryType="p",
      use_temperatureInput=true,
      TFixed=303.15)
      annotation (Placement(transformation(extent={{-102,238},{-94,258}})));
    TIL.GasComponents.HydraulicResistors.HydraulicResistor hydraulicResistor(
        zeta_fixed=0.7, hydraulicDiameter=0.8)
      annotation (Placement(transformation(extent={{-60,246},{-48,250}})));
    TIL.GasComponents.Boundaries.Boundary boundary1(boundaryType="p")
      annotation (Placement(transformation(extent={{-4,-10},{4,10}},
          rotation=90,
          origin={-22,222})));
    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary3(
                                                                     use_nInput=
         true)
      annotation (Placement(transformation(extent={{-94,250},{-86,270}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(y_start=10, T=20)
      annotation (Placement(transformation(extent={{-120,256},{-112,264}})));
    TIL.HeatExchangers.FinAndTube.GasVLEFluid.CrossFlowHX Gascooler(
      gasCellFlowType="flow B-A",
      initVLEFluid="linearEnthalpyDistribution",
      redeclare model FinMaterial = TILMedia.SolidTypes.TILMedia_Aluminum,
      m_flowVLEFluidStart=0.78,
      redeclare model WallMaterial = TILMedia.SolidTypes.TILMedia_Copper,
      redeclare model FinSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinSideHeatTransfer.Haaf,
      redeclare
        TIL3_AddOn_Supermarkt_Wurm_Februar17.Components.Gascooler.Geometry.Kl_Sup_WRG_84_119EC28V
        hxGeometry,
      redeclare model FinEfficiencyModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.FinEfficiency.ConstFinEfficiency,
      pressureStateID=2,
      hInitialVLEFluid_Cell1=260e3,
      hInitialVLEFluid_CellN=500e3,
      pVLEFluidStart=pressureState_2.pInitial,
      redeclare model TubeSidePressureDropModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSidePressureDrop.ZeroPressureDrop,
      cellOrientation="B",
      redeclare model WallHeatConductionModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.WallHeatTransfer.ConstantR
          (constantR=1e-8),
      redeclare model TubeSideHeatTransferModel =
          TIL.HeatExchangers.FinAndTube.TransportPhenomena.TubeSideHeatTransfer.ShahChenGnielinskiDittusBoelter
          (alpha_initial=5e3),
      TInitialWall(displayUnit="K") = 340,
      nCells=10)
      annotation (Placement(transformation(extent={{-36,188},{-8,216}})));

    TIL.OtherComponents.Mechanical.RotatoryBoundary rotatoryBoundary1(
                                                                     use_nInput=
         true)
      annotation (Placement(transformation(extent={{-94,220},{-86,240}})));
    TIL.GasComponents.HydraulicResistors.HydraulicResistor hydraulicResistor1(
        zeta_fixed=0.7, hydraulicDiameter=0.8)
      annotation (Placement(transformation(extent={{-62,214},{-50,218}})));
    TIL.GasComponents.Boundaries.Boundary boundary2(
                                                   boundaryType="p",
      use_temperatureInput=true,
      TFixed=303.15)
      annotation (Placement(transformation(extent={{-102,206},{-94,226}})));
    TIL.GasComponents.JunctionElements.VolumeJunction junction2(volume=1e-3,
        fixedInitialPressure=false)
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},
          rotation=90,
          origin={-40,216})));
    TIL3_AddOn_Supermarkt_Wurm_Februar17.Extremum_Seeking_Control.AlgebraischAnsatz2.FanConstantEta
      Fan2(
      use_mechanicalPort=true,
      V_flow_Start=15,
      V_flow_nominal=4,
      n_nominal=11.5,
      dp_nominal=60,
      V_flow0=6,
      deltaV_flow=1,
      eta=0.35) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={-76,216})));
    Modelica.Blocks.Interfaces.RealInput unFan
      annotation (Placement(transformation(extent={{-338,260},{-298,300}}),
          iconTransformation(extent={{-338,260},{-298,300}})));
    Modelica.Blocks.Interfaces.RealInput uVentil
      annotation (Placement(transformation(extent={{-342,130},{-302,170}}),
          iconTransformation(extent={{-342,130},{-302,170}})));
    Modelica.Blocks.Interfaces.RealInput uTUmgebung
      annotation (Placement(transformation(extent={{-336,230},{-296,270}}),
          iconTransformation(extent={{-336,230},{-296,270}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder3(
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=300,
      T=100)
      annotation (Placement(transformation(extent={{-162,246},{-154,254}})));
    TIL.VLEFluidComponents.Boundaries.BoundaryOverdetermined
      boundaryOverdetermined(
      m_flowFixed=-0.3,
      hFixed=520e3,
      pFixed(displayUnit="Pa") = 77e5,
      boundaryType="p, V_flow",
      V_flowFixed=-0.0021)
      annotation (Placement(transformation(extent={{92,192},{100,212}})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint1(stateViewerIndex=5)
      annotation (Placement(transformation(extent={{-242,124},{-234,132}})));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_3(
        pInitial(displayUnit="Pa") = 34.9e5, pressureStateID=3) annotation (
        Placement(transformation(
          extent={{-6.0,-6.0},{6.0,6.0}},
          rotation=90,
          origin={-258,100})));
    TIL.VLEFluidComponents.Valves.OrificeValve MTValve(
        use_effectiveFlowAreaInput=false, effectiveFlowAreaFixed=5.4e-6)
                                                   annotation (Placement(
          transformation(
          extent={{-8.0,-4.0},{8.0,4.0}},
          rotation=0,
          origin={-160,-20})));
    TIL.OtherComponents.Thermal.HeatBoundary
      heatBoundaryWithInputs_MTCabinet_Einzeln(
      TFixed(displayUnit="K") = 273.15 + 21,
      boundaryType="Q_flow",
      Q_flowFixed(displayUnit="kW") = -50000,
      use_heatFlowRateInput=false)            annotation (Placement(
          transformation(
          extent={{-4.0,-6.0},{4.0,6.0}},
          rotation=0,
          origin={-34,-34})));
    TIL.VLEFluidComponents.JunctionElements.VolumeJunction junction_from_IHX(
      volume=1e-2,
      hInitial=451e3,
      pressureStateID=5)
                      annotation (Placement(transformation(
          extent={{-4.0,-4.0},{4.0,4.0}},
          rotation=90,
          origin={78,64})));
    TIL.VLEFluidComponents.Valves.OrificeValve RecValve(
        use_effectiveFlowAreaInput=false, effectiveFlowAreaFixed=9.5e-6)
                                                   annotation (Placement(
          transformation(
          extent={{-8.0,-4.0},{8.0,4.0}},
          rotation=0,
          origin={-124,64})));
    TIL.VLEFluidComponents.Sensors.Sensor_p sensor_MP(useTimeConstant=true,
        initialSensorValue=3490000) annotation (Placement(transformation(
          extent={{-4.0,-4.0},{4.0,4.0}},
          rotation=0,
          origin={-170,84})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint10(stateViewerIndex=
          10)
      annotation (Placement(transformation(extent={{-144,72},{-136,80}})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint7(stateViewerIndex=7)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=0,
          origin={-130,-8})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint2(stateViewerIndex=1)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=90,
          origin={68,42})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint6(stateViewerIndex=6)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=270,
          origin={-196,4})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint9(stateViewerIndex=9)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=0,
          origin={-80,74})));
    TIL.VLEFluidComponents.Sensors.Sensor_superheating sensor_MT(
      useTimeConstant=true,
      initialSensorValue=8,
      tau=1)
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},
          rotation=90,
          origin={66,94})));
    TIL.VLEFluidComponents.Sensors.StatePoint statePoint8(stateViewerIndex=8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=0,
          origin={2,-6})));
    TIL.VLEFluidComponents.Tubes.Tube tube(
      pressureStateID=5,
      redeclare TIL.VLEFluidComponents.Tubes.Geometry.Tube6x1 tubeGeometry,
      enableHeatPorts=true,
      nCells=3,
      redeclare model TubeSideHeatTransferModel =
          TIL.VLEFluidComponents.Tubes.TransportPhenomena.HeatTransfer.ConstantAlpha
          (constantAlpha=5000),
      falseDynamicsTimeConstant=2,
      initVLEFluid="constantTemperature",
      pressureDropInitialVLEFluid=0,
      fixedPressureDropInitialVLEFluid=false,
      TInitialVLEFluid=268.15,
      TInitialWall=268.15,
      pStart=2800000)
      annotation (Placement(transformation(extent={{-24,-18},{-8,-22}})));
    TIL.VLEFluidComponents.Separators.IdealSeparator MPReciever(
      vleFluidType=sim.vleFluidType1,
      mdotStart=1,
      pressureStateID=3,
      initialFillingLevel=0.3,
      hStart=223e3,
      V(displayUnit="l") = 0.4) annotation (Placement(transformation(
          extent={{-6.0,-10.0},{6.0,10.0}},
          rotation=0,
          origin={-254,60})));
    TIL.VLEFluidComponents.Boundaries.BoundaryUnderdetermined
      boundaryUnderdetermined annotation (Placement(transformation(
          extent={{-4,-10},{4,10}},
          rotation=90,
          origin={80,152})));
    TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState_5(
      vleFluidType=sim.vleFluidType1,
      pressureStateID=5,
      pInitial(displayUnit="Pa") = 28.004e5,
      fixedInitialPressure=false)            annotation (Placement(transformation(
          extent={{6.0,6.0},{-6.0,-6.0}},
          rotation=270,
          origin={80,134})));
    TIL.VLEFluidComponents.Tubes.Tube tube1(
      falseDynamicsTimeConstant=2,
      initVLEFluid="constantTemperature",
      pressureDropInitialVLEFluid=0,
      fixedPressureDropInitialVLEFluid=false,
      redeclare TIL.VLEFluidComponents.Tubes.Geometry.TubeGeometry tubeGeometry(
        innerDiameter=innerdiameter,
        wallThickness=0.01,
        length=length_aftergascooler),
      enableHeatPorts=false,
      redeclare model TubeSideHeatTransferModel =
          TIL.VLEFluidComponents.Tubes.TransportPhenomena.HeatTransfer.ConstantAlpha
          (constantAlpha=5000),
      pStart=pHochdruckStart,
      pressureStateID=2,
      TInitialVLEFluid(displayUnit="K") = 300,
      TInitialWall(displayUnit="K") = 300,
      nCells=3)
      annotation (Placement(transformation(extent={{-156,204},{-172,200}})));
    TIL.VLEFluidComponents.Boundaries.BoundaryUnderdetermined
      boundaryUnderdetermined1 annotation (Placement(transformation(
          extent={{-4,-10},{4,10}},
          rotation=90,
          origin={-220,92})));
    TIL.VLEFluidComponents.Valves.LinearDirectionalControlValve
      linearDirectionalControlValve
      annotation (Placement(transformation(extent={{-266,76},{-250,88}})));
  equation

    connect(sensor_HP.port,HPValve. portA) annotation (Line(
        points={{-242,178},{-258,178},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint4.sensorPort,HPValve. portA) annotation (Line(
        points={{-258,206},{-258,156}},
        color={153,204,0},
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
    connect(statePoint3.sensorPort,pressureState_2. portB) annotation (Line(
        points={{30,206},{30,202},{42,202}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_T.port,HPValve. portA) annotation (Line(
        points={{-192,210},{-192,202},{-258,202},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pressureState_2.portB, Gascooler.portB_vle) annotation (Line(
        points={{42,202},{-8,202}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Gascooler.portA_gas, boundary1.port) annotation (Line(
        points={{-22,216},{-22,222}},
        color={255,153,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Fan1.rotatoryFlange, rotatoryBoundary3.rotatoryFlange) annotation (
        Line(
        points={{-76,254},{-76,260},{-90,260}},
        color={135,135,135},
        thickness=0.5,
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
    connect(Fan2.rotatoryFlange, rotatoryBoundary1.rotatoryFlange) annotation (
        Line(
        points={{-76,222},{-76,230},{-90,230}},
        color={135,135,135},
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
        points={{-40,212},{-40,180},{-22,180},{-22,188}},
        color={255,153,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(rotatoryBoundary3.n_in, firstOrder.y) annotation (Line(
        points={{-94,260},{-111.6,260}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(rotatoryBoundary1.n_in, firstOrder.y) annotation (Line(
        points={{-94,230},{-104,230},{-104,260},{-111.6,260}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(firstOrder.u, unFan) annotation (Line(
        points={{-120.8,260},{-224,260},{-224,280},{-318,280}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(HPValve.effectiveFlowArea_in, uVentil) annotation (Line(
        points={{-263,148},{-292,148},{-292,150},{-322,150}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(uTUmgebung, firstOrder3.u) annotation (Line(points={{-316,250},{
            -316,250},{-162.8,250}},  color={0,0,127}));
    connect(firstOrder3.y, boundary.T_in) annotation (Line(points={{-153.6,250},
            {-118,250},{-118,242},{-102,242}},
                                         color={0,0,127}));
    connect(firstOrder3.y, boundary2.T_in) annotation (Line(points={{-153.6,250},
            {-118,250},{-118,210},{-102,210}},color={0,0,127}));
    connect(pressureState_2.portA, boundaryOverdetermined.port) annotation (
        Line(
        points={{54,202},{76,202},{96,202}},
        color={153,204,0},
        thickness=0.5));
    connect(statePoint1.sensorPort, HPValve.portB) annotation (Line(
        points={{-238,124},{-248,124},{-248,120},{-258,120},{-258,140}},
        color={153,204,0},
        thickness=0.5));
    connect(pressureState_3.portA, HPValve.portB) annotation (Line(
        points={{-258,106},{-258,123},{-258,140}},
        color={153,204,0},
        thickness=0.5));
    connect(MPReciever.portGas,RecValve. portA) annotation (Line(
        points={{-249,64},{-132,64}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_MP.port,MPReciever. portGas) annotation (Line(
        points={{-170,80},{-170,64},{-249,64}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint2.sensorPort,junction_from_IHX. portA) annotation (Line(
        points={{72,42},{78,42},{78,60}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint10.sensorPort,RecValve. portA) annotation (Line(
        points={{-140,72},{-140,64},{-132,64}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint7.sensorPort,MTValve. portB) annotation (Line(
        points={{-130,-12},{-130,-20},{-152,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(MPReciever.portLiquid,MTValve. portA) annotation (Line(
        points={{-254,50},{-254,-20},{-168,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint6.sensorPort,MTValve. portA) annotation (Line(
        points={{-200,4},{-200,-20},{-168,-20}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(RecValve.portB,junction_from_IHX. portB) annotation (Line(
        points={{-116,64},{74,64}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint9.sensorPort,junction_from_IHX. portB) annotation (Line(
        points={{-80,70},{-80,64},{74,64}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(tube.portB,junction_from_IHX. portA) annotation (Line(
        points={{-8,-20},{78,-20},{78,60}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(statePoint8.sensorPort,junction_from_IHX. portA) annotation (Line(
        points={{2,-10},{2,-20},{78,-20},{78,60}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(heatBoundaryWithInputs_MTCabinet_Einzeln.heatPort,tube. heatPort[
      1]) annotation (Line(
        points={{-34,-34},{-16,-34},{-16,-21.3333}},
        color={204,0,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(sensor_MT.port, junction_from_IHX.portC) annotation (Line(
        points={{70,94},{70,94},{78,94},{78,68}},
        color={153,204,0},
        thickness=0.5));
    connect(MTValve.portB, tube.portA) annotation (Line(
        points={{-152,-20},{-88,-20},{-24,-20}},
        color={153,204,0},
        thickness=0.5));
    connect(junction_from_IHX.portC, pressureState_5.portB) annotation (Line(
        points={{78,68},{78,128},{80,128}},
        color={153,204,0},
        thickness=0.5));
    connect(boundaryUnderdetermined.port, pressureState_5.portA) annotation (
        Line(
        points={{80,152},{80,140}},
        color={153,204,0},
        thickness=0.5));
    connect(tube1.portA, Gascooler.portA_vle) annotation (Line(
        points={{-156,202},{-98,202},{-36,202}},
        color={153,204,0},
        thickness=0.5));
    connect(tube1.portB, HPValve.portA) annotation (Line(
        points={{-172,202},{-258,202},{-258,156}},
        color={153,204,0},
        thickness=0.5));
    connect(pressureState_3.portB, linearDirectionalControlValve.portA)
      annotation (Line(
        points={{-258,94},{-258,88}},
        color={153,204,0},
        thickness=0.5));
    connect(MPReciever.portInlet, linearDirectionalControlValve.portC)
      annotation (Line(
        points={{-259,64},{-266,64},{-266,66},{-274,66},{-274,80},{-266,80}},
        color={153,204,0},
        thickness=0.5));
    connect(linearDirectionalControlValve.portB, boundaryUnderdetermined1.port)
      annotation (Line(
        points={{-250,80},{-234,80},{-234,92},{-220,92}},
        color={153,204,0},
        thickness=0.5));
    annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-340,-40},{100,300}},
          initialScale=0.1)),
      experiment(StopTime=1.3e5, __Dymola_Algorithm="Dassl"),
      __Dymola_experimentSetupOutput(equdistant=false),
      Icon(coordinateSystem(extent={{-340,-40},{100,300}},
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
  end Fan_Pressure2;

  model Fan_Pressure2_Tester
    Kreislauf.Fan_Pressure2 julius(pHochdruckStart=77e5, HPValve(
          effectiveFlowAreaTypical=4e-6))
      annotation (Placement(transformation(extent={{-28,-2},{22,34}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15 + 25)
      annotation (Placement(transformation(extent={{-68,8},{-48,28}})));
    Modelica.Blocks.Sources.Ramp ramp(
      offset=1e-4,
      height=-1e-4 + 4e-6,
      duration=8,
      startTime=2)
      annotation (Placement(transformation(extent={{-70,-16},{-50,4}})));
    Modelica.Blocks.Sources.Step step(
      height=10,
      offset=10,
      startTime=100)
      annotation (Placement(transformation(extent={{-70,32},{-50,52}})));
  equation
    connect(julius.uTUmgebung, realExpression1.y) annotation (Line(points={{
            -25.2727,28.7059},{-37,28.7059},{-37,18},{-47,18}}, color={0,0,127}));
    connect(ramp.y, julius.uVentil) annotation (Line(points={{-49,-6},{-38,-6},
            {-38,18.1176},{-25.9545,18.1176}}, color={0,0,127}));
    connect(step.y, julius.unFan) annotation (Line(points={{-49,42},{-42,42},{
            -42,38},{-25.5,38},{-25.5,31.8824}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}})));
  end Fan_Pressure2_Tester;
  annotation (uses(
      TIL(version="3.4.1"),
      Modelica(version="3.2.1"),
      TILMedia(version="3.4.1"),
      TIL3_AddOn_Supermarkt_Wurm_Februar17(version="1")));
end Kreislauf;
