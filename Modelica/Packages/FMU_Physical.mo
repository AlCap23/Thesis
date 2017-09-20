within ;
package FMU_Physical "Package for Physical Systems"
  package HP_HT_Control
    "Package for controlling the high pressure and temperature"

    model System "Physical System"
      Supermarktmodelle_342.Identification_Physical_System.Inputs
        Physical_System
        annotation (Placement(transformation(extent={{-30,-18},{34,22}})));
      Modelica.Blocks.Interfaces.RealInput u_1 "Temperature"
        annotation (Placement(transformation(extent={{-120,30},{-80,70}})));
      Modelica.Blocks.Interfaces.RealInput u_2 "Valve"
        annotation (Placement(transformation(extent={{-120,60},{-80,100}})));
      Modelica.Blocks.Interfaces.RealOutput T_H
        annotation (Placement(transformation(extent={{96,6},{116,26}})));
      Modelica.Blocks.Interfaces.RealOutput p_H
        annotation (Placement(transformation(extent={{96,-14},{116,6}})));
      Modelica.Blocks.Sources.RealExpression T_A(y=273.15)
        "Ambient Temperature"
        annotation (Placement(transformation(extent={{-100,6},{-80,26}})));
      Modelica.Blocks.Sources.RealExpression P_M(y=35e5) "Medium Pressure"
        annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      Modelica.Blocks.Sources.RealExpression Q_C(y=50e3)
        "Heat flow into the system"
        annotation (Placement(transformation(extent={{-100,-26},{-80,-6}})));
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
      connect(u_1, Physical_System.nFan) annotation (Line(points={{-100,50},{
              -62,50},{-62,44},{-15,44},{-15,22}}, color={0,0,127}));
      connect(u_2, Physical_System.Ventil) annotation (Line(points={{-100,80},{
              -48,80},{-48,74},{7.8,74},{7.8,22}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end System;
  end HP_HT_Control;
  annotation (uses(Supermarktmodelle_342(version="1"), Modelica(version="3.2.2")));
end FMU_Physical;
