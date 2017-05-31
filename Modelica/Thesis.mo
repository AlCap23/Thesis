package Thesis "Package for the Master Thesis"

  package InteractionStudy
    "Studies the influence of the interaction from valve to temperature and fan to pressure in different configurations"
    model OpenSystem "System - Valve and Gascooler"
      TIL.VLEFluidComponents.Boundaries.BoundaryOverdetermined
        CompressorOutlet_Boundary(
        vleFluidType=sim.vleFluidType1,
        use_massFlowRateInput=true,
        hFixed=500e3,
        pFixed=8000000)
        annotation (Placement(transformation(extent={{54,22},{46,42}})));
      TIL.VLEFluidComponents.Boundaries.Boundary ValveOutllet_Boundary(
        streamVariablesInputType="h",
        hFixed=200e3,
        boundaryType="p",
        pFixed=2000000) annotation (Placement(transformation(
            extent={{-4,-10},{4,10}},
            rotation=90,
            origin={-56,-32})));
      TIL.VLEFluidComponents.Valves.OrificeValve valve(effectiveFlowAreaFixed=
            0.3e-6) annotation (Placement(transformation(
            extent={{-8,-4},{8,4}},
            rotation=90,
            origin={-56,-4})));
      TIL.GasComponents.Boundaries.BoundaryUnderdetermined
        Airflow_Outlet_Boundary annotation (Placement(transformation(
            extent={{-4,-10},{4,10}},
            rotation=90,
            origin={0,2})));
      TIL.GasComponents.Boundaries.Boundary Airflow_Inlet_Boundary(
        boundaryType="V_flow",
        gasType=sim.gasType1,
        hFixed=300,
        streamVariablesInputType="T",
        use_temperatureInput=true,
        V_flowFixed=-10) annotation (Placement(transformation(
            extent={{4,-10},{-4,10}},
            rotation=90,
            origin={0,70})));
      TIL.VLEFluidComponents.Sensors.StatePoint ValveOutlet
        annotation (Placement(transformation(extent={{-42,-22},{-34,-14}})));
      TIL.VLEFluidComponents.Sensors.StatePoint GascoolerOutlet
        annotation (Placement(transformation(extent={{-44,40},{-36,48}})));
      TIL.VLEFluidComponents.Sensors.StatePoint GascoolerEntry
        annotation (Placement(transformation(extent={{36,40},{44,48}})));
      TIL.GasComponents.Sensors.StatePoint FanOutlet
        annotation (Placement(transformation(extent={{18,54},{26,62}})));
      inner TIL.SystemInformationManager
                                     sim(redeclare
          TILMedia.VLEFluidTypes.TILMedia_CO2 vleFluidType1, redeclare
          TILMedia.GasTypes.TILMedia_MoistAir gasType1)
        annotation (Placement(transformation(extent={{64,62},{84,82}})));
      TIL.HeatExchangers.MPET.GasVLEFluid.CrossFlowHX
                  crossFlowHX(
        hInitialVLEFluid=350e3,
        pressureStateID=1,
        m_flowVLEFluidStart=0.05,
        initVLEFluid="linearEnthalpyDistribution",
        redeclare model WallHeatConductionModel =
            TIL.HeatExchangers.MPET.TransportPhenomena.WallHeatTransfer.ConstantR
            (constantR=1e-4),
        redeclare model FinSideHeatTransferModel =
            TIL.HeatExchangers.MPET.TransportPhenomena.FinSideHeatTransfer.Chang,
        redeclare TIL.HeatExchangers.MPET.Geometry.Example hxGeometry,
        nCellsPerPass=5,
        redeclare model TubeSidePressureDropModel =
            TIL.HeatExchangers.MPET.TransportPhenomena.TubeSidePressureDrop.QuadraticMassFlowDependent
            (mdot_nominal=0.2, pressureDrop_nominal=5000),
        redeclare model FinEfficiencyModel =
            TIL.HeatExchangers.MPET.TransportPhenomena.FinEfficiency.ConstFinEfficiency,
        redeclare model WallMaterial =
            TILMedia.SolidTypes.TILMedia_Aluminum,
        redeclare model FinMaterial =
            TILMedia.SolidTypes.TILMedia_Aluminum,
        redeclare model FinSidePressureDropModel =
            TIL.HeatExchangers.MPET.TransportPhenomena.FinSidePressureDrop.KimBullard,
        hInitialVLEFluid_CellN=250e3,
        pressureDropInitialVLEFluid(displayUnit="Pa") = 5000,
        hInitialVLEFluid_Cell1=430e3,
        redeclare model TubeSideHeatTransferModel =
            TIL.HeatExchangers.MPET.TransportPhenomena.TubeSideHeatTransfer.ConstTwoPhaseGnielinskieDittusBoelter
            (alpha_initial=1000, alpha_twoPhase=4500),
        pVLEFluidStart=300000,
        TInitialWall=318.15)            annotation (Placement(transformation(extent={{-14,18},
                {14,46}},          rotation=0)));
      TIL.VLEFluidComponents.PressureStateElements.PressureState pressureState(
                                                             pressureStateID=1, pInitial(
            displayUnit="Pa") = 80e5)
        annotation (Placement(transformation(extent={{-34,26},{-22,38}},rotation=0)));
      Modelica.Blocks.Sources.Step step(height=0, offset=-5)
        annotation (Placement(transformation(extent={{-42,74},{-22,94}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=-0.5)
        annotation (Placement(transformation(extent={{92,24},{72,44}})));
    equation
      connect(valve.portA, ValveOutllet_Boundary.port) annotation (Line(
          points={{-56,-12},{-56,-22},{-56,-32}},
          color={153,204,0},
          thickness=0.5));
      connect(ValveOutlet.sensorPort, ValveOutllet_Boundary.port) annotation (
          Line(
          points={{-38,-22},{-48,-22},{-56,-22},{-56,-32}},
          color={153,204,0},
          thickness=0.5));
      connect(GascoolerEntry.sensorPort, CompressorOutlet_Boundary.port)
        annotation (Line(
          points={{40,40},{40,32},{50,32}},
          color={153,204,0},
          thickness=0.5));
      connect(Airflow_Inlet_Boundary.port, crossFlowHX.portA_gas) annotation (
          Line(
          points={{0,70},{0,46}},
          color={255,153,0},
          thickness=0.5));
      connect(FanOutlet.port, crossFlowHX.portA_gas) annotation (Line(
          points={{22,54},{0,54},{0,46}},
          color={255,153,0},
          thickness=0.5));
      connect(crossFlowHX.portB_vle, CompressorOutlet_Boundary.port)
        annotation (Line(
          points={{14,32},{50,32}},
          color={153,204,0},
          thickness=0.5));
      connect(Airflow_Outlet_Boundary.port, crossFlowHX.portB_gas) annotation (
          Line(
          points={{0,2},{0,12},{0,16},{0,18}},
          color={255,153,0},
          thickness=0.5));
      connect(GascoolerOutlet.sensorPort, valve.portB) annotation (Line(
          points={{-40,40},{-40,32},{-56,32},{-56,4}},
          color={153,204,0},
          thickness=0.5));
      connect(crossFlowHX.portA_vle, pressureState.portA) annotation (Line(
          points={{-14,32},{-22,32}},
          color={153,204,0},
          thickness=0.5));
      connect(pressureState.portB, valve.portB) annotation (Line(
          points={{-34,32},{-34,32},{-56,32},{-56,4}},
          color={153,204,0},
          thickness=0.5));
      connect(step.y, Airflow_Inlet_Boundary.T_in) annotation (Line(points={{
              -21,84},{-8,84},{6,84},{6,74}}, color={0,0,127}));
      connect(CompressorOutlet_Boundary.m_flow_in, realExpression.y)
        annotation (Line(points={{54,34},{64,34},{71,34}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end OpenSystem;
  end InteractionStudy;
  annotation (
    uses(Modelica(version = "3.2.2"),
      TIL(version="3.4.1"),
      TILMedia(version="3.4.1")));
end Thesis;
