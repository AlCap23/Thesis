package TIL3_AddOn_Supermarkt_Wurm_Februar17
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
        pi=portB.p/portA.p;

        eta=C-K*pi;

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
          annotation (Placement(transformation(extent={{-90,-10},{-70,10}}, rotation=0)));

      protected
        TIL.Internals.GetInputsRotary getInputsRotary
          annotation (Placement(transformation(extent={
                  {-34,-10},{-14,10}}, rotation=0)));

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

        parameter Boolean steadyStateMomentum = false
          "true, to avoid differentiation of angular speed"
          annotation(Dialog(enable = use_mechanicalPort));

        parameter SI.Inertia J = 5e-3 "Moment of inertia of the fan"
          annotation(Dialog(enable=use_mechanicalPort));

        parameter SI.Frequency nFixed(min=0) = 50 "Constant speed"
          annotation(Dialog(enable = not use_mechanicalPort));

      //____________________ Energy balance _____________________

        parameter Boolean isenthalpicProcess=false "= true, if h_out = h_in"
          annotation(Dialog(tab="Advanced",group="Energy balance"));

        parameter SI.Temperature maxDeltaT( displayUnit="K") = 5
          "Maximal possible temperature difference from portA to portB, when m_flow is almost zero."
          annotation(Dialog(tab="Advanced",group="Energy balance"));

      protected
        final parameter Real dh_dhmax_transition = 0.9
          "Transition point in percentage of maximal possible enthalpy difference from portA to portB."
          annotation(Dialog(tab="Advanced",group="Energy balance"));

        SI.SpecificEnthalpy dh "Enthalpy difference from portA to portB.";

      //__________________   Fan characteristic   __________________
      public
        parameter SI.Frequency n_nominal = 50 "Nominal value for fan speed"
          annotation(Dialog(group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.Pressure dp_nominal( displayUnit="Pa") = 360
          "Nominal value for pressure increase @ n_nominal"
          annotation(Dialog(group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.VolumeFlowRate V_flow_nominal = 0.1
          "Nominal value for volume flow rate @ n_nominal"
          annotation(Dialog(group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.VolumeFlowRate V_flow0 = 0.21
          "Volume flow rate @ dp = 0 and @ n_nomial"
          annotation(Dialog(group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.VolumeFlowRate deltaV_flow(min=0) = 0.07
          "Volume flow rate difference between V_flow0 and gradient line @ nominal point"
          annotation(Dialog(group="Fan Characteristic @ nominal gas conditions"));

        parameter SI.Temperature T_nominal = 298.15
          "Nominal value for gas temperature"
          annotation(Dialog(group="Nominal gas conditions used for Fan Characteristic"));

        parameter SI.Pressure p_nominal = 1.013e5
          "Nominal value for gas pressure"
          annotation(Dialog(group="Nominal gas conditions used for Fan Characteristic"));

        parameter Real[gasType.nc] mixingRatio_nominal = gasType.defaultMixingRatio
          "Nominal value for gas mixing ratio"
          annotation(Dialog(group="Nominal gas conditions used for Fan Characteristic"));

      protected
        SI.HeatFlowRate Q_flow_loss "Heat flow rate of fan losses to ambient";

      //____________________   Losses an Efficiencies   _____________________
      public
        parameter SI.Efficiency eta = 0.3
          "Fan efficiency @ maximum hydraulic power and nominal conditions => V_flow_maxPhyd"
          annotation(Dialog(tab="Fan Power", group="Losses and Efficiencies @ nominal conditions"));

      //____________________ Gas Properties _____________________
      protected
        final parameter SI.Density d_nominal(start=1)=
          TILMedia.GasFunctions.density_pTxi(gasType, p_nominal, T_nominal, mixingRatio_nominal)
          annotation(Evaluate=true);

      /********************************************************/
      /*                  Initialization                      */
      /********************************************************/
      public
        parameter SI.Pressure dpInitial = 500
          "Initialization of pressure inrease"
           annotation(Dialog(tab="Start Values and Initialization", group="Initialization"));

        parameter SI.VolumeFlowRate V_flow_Start = 1
          "Start value for volume flow rate"
           annotation(Dialog(tab="Start Values and Initialization", group="Start Value"));

      equation
        assert(V_flow_nominal < V_flow0, "Parameter V_flow_nominal has to be smaller than V_flow0.");

        d_gas = gas.d;

        portA.m_flow = volumeFlowRate*d_gas;

      //____________________ Affinity laws _____________________

      //  V_flow_affinity = V_flow_nominal *n/n_nominal;
      //  dp_affinity = dp_nominal *(n/n_nominal)^2*(d_gas/d_nominal);

        coef_affinity.dp_0 = coef.dp_0 *d_gas/d_nominal *(n/n_nominal)^2;
        coef_affinity.c1 = coef.c1 *d_gas/d_nominal *n/n_nominal;
        coef_affinity.c2 = coef.c2 *d_gas/d_nominal;
        coef_affinity.V_flow_transition = coef.V_flow_transition *n/n_nominal;

      //____________________ Fan characteristic _____________________

      // Equation without mathematical optimization for modelica:
      // quadratic:    dp = V_flow * (c2*V_flow + c1) + dp_0
      // or linear:    dp = dp_transition + (V_flow - V_flow_transition)*ddp_dV_flow_transition

        dp = TIL.GasComponents.Fans.Internals.semiSquareFunction(volumeFlowRate,
          coef_affinity);

      //____________________ Power calculation _____________________

        P_loss = P_hyd*(1/eta - 1);

        P_hyd = dp*volumeFlowRate;

        P_shaft  = P_loss + P_hyd;

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

        fluidTorque = if noEvent(w<1e-8) then 0 else P_shaft/w;

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

        connect(rotatoryFlange, getInputsRotary.rotatoryFlange) annotation (Line(
              points={{-80,0},{-34,0}}, color={135,135,135}));

                  annotation (
          Images(
            Dialog(
            tab="General",
            group="Fan Characteristic @ nominal gas conditions",
            source="./Images/FanCharacteristic.png"),
            Dialog(
            tab="Fan Power",
            group="Losses and Efficiencies @ nominal conditions",
            source="./Images/FanPowerLosses.png")),
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
            "5h64F+7pJIw9BycV95wMPs/CM/FsPCPPyjNj9EmTsBasCWujt6+xX4UhX/Saffb+ITN9jADwTEUU4CnnArTGCC66Afv2vVQrkFIEKTLwYIS8KXQj781gGzxjWt4wmnjLeM3Mua9GCSqkIBEDyAGnFvLGA8c4814kesB30ve5Vrouv8GblEVBTKrGnvvjXpPB5xl4Fp6JZ+MZeVaiITx7Mvr272v427bX++7995kAQAJOnA6oUmmbUqnez7g0AXnvFB0gF47RpLUQr5k594TNEymgUj5FCjC8tM2liEGKGlyQBGoMMNqzvEf1CIVxTwY+GXl+g99KRXvcAwTlOIgK95Y8fO6Ze+cZUmifZ8PoG953X7Z5X6Z7q0z9o/y/l85yLx+qFFYRBfh8zG02h6jCabvCqZIBogOprTARAoriCJcnUkDenCmE1BFgdPG0YzhJYYjpm8coY5zxxiEJRA9SBCEZ8HH/TJ/jO3w3GXiuybX5DX6L3+S3uQfuBWNPOoOqfTx8SEzV4PNMKFRz+u7Ftu/FVPmP7Shf2JJe2spePlRFWMdEAR7/8S1D4dYEdAoDdUJACxzhcjxoDCshdPriMbYYXULr5NQxxOTX8cAJu9NOh6EmDE8EAcN93ZvP8Fm+w3e5Btfimlyb3+C3kqHnHriXZOy5R+5Vg6+h74Khn3SPOXj/fU8BXEwH/OLbbvOkQAlApwjAOMVUJQUoqBQpSMSAQrpEDvDAeWOgiR5AFAjFz/Lms3yH76brJCPPb6TcPVGKZOj17jX4XTb49dQcNqN8bffV+8+BABRnBHzrhfMiCmAqQCXVFyU1rpYgdRokcoBxxhvnjbGe552+xzXqRt4wvvuoj/sohf6xFdgMbEefjX8uBAAGd/7cHYdGAYwCdD4K0FfF63NJKtqAAcgttiIH7z8XAkAU4AYC/fTBtlEASYAkQAyIATFwBQNEiLER5Qub0fcaud7OAagLbosoAOMcYXhtYJreg3IQA2JADLQHA9iGcuQv8X9shgSgR4twALN7+nDPKIDsXxIoBsSAGLjAAN7/k79cdI7zwlb03vjnlAJIwjxzOFB7GLfej7IQA2KgDRiotP2d5WL8cyQABcX70ttuMwog+9cDFANiQAwUtgCbUL56O/RnHLHJIsxRe/BjWjye+BkLAtvAvL0HPUAxIAY2hQGMP7agbPs7ycn7zzECcDEc6KsPH1sQKPvXAxQDYiBjDFD4hy0oX70e+mME4GZhR9EWaEHgeM/jiZ/YGn75zsMh/9wUM/d39QrFwPIYYA8/f/tt7uUxJKdW+JdF21+dBOSYAuCZafEoCgI9J+CyknkulMV/G03BGn7zqdPhpyUBkqCMPcQukxD2LnuYF3v6S288EMtVLMfEP2wAtqC0CdnZw+weuMKA9pD88zH1yRHBg+Hnf2Nv+JfPFZvh0utFSYBKUwLQOQxg/Nm79Rd7/HO/stu552maiKHz0f3lC1uQpS3M8qErwj4GAJ99zU62JOCzP7s9fOGTJ3U9cR7/B72w/LMgBme/sJO90mhaCXm95UPcruHVNawZf/Yw4e1iL6fX1yPvzd7Pcf0w/uj88oXyy9YOZvvgpdAp+jj/8ydPsysITHn+uuUvlcVWHAkLNtglheKgSlYSoMHK0WB06ZnZo2VFO9uWvbsTRzuntOdhfb+T8sut1ofCP3R+uT7ZFf5VCU/uBIDnx9PNajYAucCKkkg64Tj+ZfsPXjIYvOU7B4M3x/s4/l0SoNHvkgHM+V7HGX/2MHuZPf32lxb7GYOH13vxQhc88zv7WUQDaj3/6P6sbWDWD4/w3/ndxaYo6GDfUwHk/sbk+c/i0XffEcrhraEkfu87BoM33Dp68++3f9eF0ijWyEiAhCBnI9vWZx9n/G8P41/fz+zxPxwRAfLe7P2L1wtxVkqf6wNqof/TUvdnbQOzfngIwNtGBq5ICL3Y01QAuT5yfrXXefz3AZsAI/+mfzwYvDEM/uvD8L/+28t3SQLKNdpKRInr5OIxtFXhe18SsYQB9mIlogdR32JPY/zr+5k9zl7n7+8apQau1Ad85b6jXtYHEPpHx5evnVKvZW0Ds354CACbgbB3uRF6lQogt0eOb8zrECVBePD34/l/t64oEgHgn0EC+DueA9+pkoA//e08woYaW41tWzHAHqy8CuOPYZtlT7P3yzQf+/qoeiEIRZ/qA2qh/xvofHQ/NiDnd9YPj+AJdbMR+pYKqHkFaW+fxL9skwskL5jCgxcef9Xwz0gC8Bbaqhy9Lw13nzHwZzGsq278IerXGv9KdC+l+agPIAUY19qNNzri4kXKkBbhLq/juNA/Op/nz9n48+zZLwAeLmGxeirg0zEkoougJ4dHLq/2Oov/3iP3V83zXwoPTjL+FYWBYkFZlGTpwmOABDgwSGPbxf3SxXvGoLHnKi/+Y2tm418j9xhCvGF0YLm3CSugMy5etAo/2dFW4HGh/5TulABkHgIpvN9yA1RTAV0bEESev6YU2Lzn8b5Brm9inn+a4a8pCzbOOBLgwCAJQBeNadfuecyAnyOMNntyJs//mijfRX3AKCVKWmBsfUCX2gZrA38uQv/J+ZEASACKordxqYAnw5vuwpRAcnVj2voKr2CmPP8CJIAQWhk2vEhC2iEgCeiaQe3S/VLpX+viOSCqRzrvSgHvPHu68ln0IESCa5YO0XbokeNqNIB93oWxwsWs/9Dh5auo+k+h/5T2lABIAEZV71dTAecAvc2pgAnje08C1EVbH17BRZ6/Wt2/oHKodgckwlT2FkMCiDYURMQOAYlAlwxrF+51TE3PPnsPg9aU8a/vb3RHrT7gtEoE2j5WGN1dcYyKqv/6WkkAJABX2t7KytgDwP61aJ9rWxSAXNyY8b3k7PZhuQCdnB5Geq48/5zEIOUO6wODWDcKlLqgWL1HyUrbMVAr9oNo77DnLvb4nPt2atFvpe4npQVq9QEF2U8vdFHbxgqjs9Hd5euAaMaVtsh4TgmABOAmAbiaCsCbHn7xDQetIAHk3sbk+blFcnVbDP4Y28+/KgVRrldlYNClNkGKES0O1MC23cC29f7YO7WCXjzw7VTPUxD8Fe7takQAIkC0AR2Dron3YZUE8O9taRvE+KOzy9fJuNC/KYBR+2P2DGjcBkqFbiXQC7bLlMBNKopp43vJ2ZG7W5tSuL5N8CjtPkJwT3v62Eaxs0nc+tuLEcDaZD+2E3uqqPQf58mugwhU6wPaPFa4ctAPunuLNEYR+h9DlowAGAG4Cow0BnfEdvfYfZwbvYl6gAnje/EErozvXWW4f6qCqRQPlaNGLyh48g40BosZA9ctr3WD7NdeF8V+S1X6NxEtqIwJr4wVptLurHrPdAVtYqwwOroy7W8vjUOepBslABKA8WG0sjWwZLpFuGud9QDXHNN7Kc/feAHQMkqiLKQkVFhWEKMYiggKL1MCeRkyict88ibFVwv5s3d22UsrKfZrYK+nscLl/IArbYPrPHa4lvc/RHdzf9c5RhIACcDEPBohL0Lr1SmBq64HmDa+F0bbOmVQUySpOLA8SIh84UkiAaQEuj5ZTMM2n2FzvaavF2myWisvUb5ipv/Ki/2WJAJEJTY9VriW9y9a/tDd01KiEgAJwORCmtKjLefg0w9beLOrqgeYZXzvxvL88yqJyhkCFe/gIkr45egS6NJAEY3YdCPmGs2/RuwB9kLtdcieWWiy37z7tInPV9IClWOHd6rEn+ejbXBVLcK1vH/R8jdLukQCIAG4vpJ2TD1A0/MByJWRM6u9zgj/LTW+t4nNvcw1KimBMpVyKSWAQqDYScMxv+Fwzbq/ZmC/tu+LkP/K+vuX2cuzfLckAkQsph073ORY4Vq//x4tkrMWSkoAJABTW2lSSLtaD0ChybLzAa4b33vtMb2zbMa2fKaiFCopgeMq2aF9SIPWfYOmDGeX4ZhTOkmTXbTyrnqGx9Si3mX0R0n8U30AzxXvlYwVRgd/4+bZJ4cMQJsnZSIBkABMJQBpVHC9HoCe/EVIQMrzr2187zKbuanvlikBQoSVLoEipcILT8h2wdkNiMa2m2s1wes/aGSef1N7tanrjB8rfFQl/+nY4UXwjO6tDEmaOe9fJT8SAAnATAQgjQou6wFgtIXx+kKcxz0PCbhmfO8OEYaVje9talMvc52KZ1BOD6Su4lLuw9qAbhq2RRR4Tt+ZkOsH+8VUv7YX9i4cMajVB1SOHb6070kHzuMAoHPRveULXVzMSJi3K0oCIAGYjQBg+EpGW4ayKXIpXmczDAlqy/jehTfyMoZ/TJcAOTo27LgCwT6cQZ6TcfNZrydtGLbaIT6ojRtpbPfFeR0N7rE27PNL91Ah/02MFUbnVl676ORZiv7q6yIBkADMTgBKEkCOqexzLygoYaw/+ZntsXnsCeN7YawbG9/bCuVwdWbAlarhdfYQa8SMPDSNAWp8wHDtVXj9reztXwcBqez7ylhhdOGl13WRQHRtJX1azPmf1u8/SedJACQA8xGAcv49gCvDWUcgl6LA+tz7Ccf0ohG2Ae1Gx/euY7NP+43aVLEyGsAYNAhS8VomR9i0Qvd6koRZMTBm74PpIte/rsO6WkH0J+iACccOn1RZwLhjh9GxlUl/R9QT1Y/4nee5JQASgLkJwKSiwDQpcELID+a/2mN6pxnctv694hVUagMuuU6mBTS+sxrfTX5uwt7HsG33Ote/iG6ZY6ww61qb9HdR9Ddpzv8sREACIAFYiACMKQo8g72OyfXB/Ns7vneRjbui7+AVpDPIy04BzmEo1jW9OHq0yR7iTRoLf7s/pOaaGp+9K7M8VrR/ZjF4rfxMrW1wXCSwplvRCQsV/VkDMDoFML09DXCZzXi1KBBjX31d5Pl7W+W7zPqN++7kHuJLC0sLZtvOINeg98egzyrLCbM8wOpo75ejfOetTp/VUPfcg6Xb6rCmU9Gx1Atlb7uaWIPsF3HWjTbxc2MmBYZgTuJtnn8ZcjC+h/i4pgyKMaqOFM7P8M5qoFf1uWvO7Liy91d5SmcTRqAD16gWCBMVzN5uNbUG2S/k0gSgUhSYzgwg15fO7e70RK9lDHgT363lCCf1EKdCQYmARGBVBj9d95ohXqehlNc+urspQ9CR6zA3JHub1eQaZL+YTRCAVBRImI8cNv2oqwr5NXW/nbrO5B7is2pEQCIgAVgVAbjG8IPBjdX4NGkMvFZ+5EIC0ISn6jUWK6Scd91mLBaSCEgEmiIC1xh+ctE33vU9gwH97LQFb4L0a7TzM9pNylwCMK8R8vPrMfbXrXNZH1A7g/zKYSNEBywWlAwsQgZScd+Y8zoKwx/vrdTWt8gEuqYicE0aA6+VH5mQAGjQN2/QF5RBdZhIOT+AquGJRIBjlxcxBn4nHxIBRiCNY16XDD9DvC5G+AYhbcqgz3sdjXZ+RrtJmUsAFjQ+825UP78iJVkpFEQpTyMC34xTB5/5nX2JQBymIrG5uQZgghkTXTH8SZ80aQy8Vn5kQgIgAdiY99IoKRpDBMjPUqAV70vFgij5/xbnNzx/+23OEsiYCBDmBwNjhncBETADdopQf1s8fgfZ5GekV0nMJAASgH4QgCTHkgiQl6VGgAKtcrIYynysi8eBLV/4jT094kzIALIec0hPcv7BSFHVD3bAEFhqazvvKo2D1+4/2ZAASAD6RQCq8qwUCzKRrRwvzFCRsUleogIMFnLUcP/SA8gU2SLjCS8wUfTxg5W2G35TAP03zusgYBIACUB/CUAlKkCLFq1aDGh6eyh5QrvxpmDwSnoAA0GtwLNvPDBF0OGoACF+ZIgsJ7yQPRjYBhNgY1PtfIumw9ZhJPyN/pINCYAEoP8EoJYeoHo7FQyW6QHGix5PshKSge5EBGYw+ogZWe8he/L7b+mg4TcC0F+jvE7CJQGQAORDAMbUCeDxcT57JSpwEBtwosuYyIBpgvaQAmQxxdPH6CNTZLuFrJF5V8L810UH1mks/K3+kQ4JgAQgPwJQqxNII5yJClD4VakVOJyUIsCiUD1OXtkCwvWTAdactZ9QwZ+COYT4keFOkduvVPNvYmrfomF+CUD/DG9byJQEQAKQNwGYEBUgH0x4uEwRUDh4LRnA4tBHTluZ0YHmCUHy8if06lezNxdGP4X4U26/zdX8i5KDthgS76ObJEUCIAGQANQxUGklTIWDY8jAxDQB1ohqc1rNIAROIJyfELBmrB1reE3lfjL8yKLw9HMw+lWyoOHtpuFti9wkABIACcCUcwfSyOEqGSjTBNtlXvm46oJO+ne8V8LWTJ0zSnCTFLAWrAlrM4OHn5aXNSenv40s0hHcyKiPnv6kCEFbDIn30U0iIgGQAEgAZsVALTJA9Tg94xSVlVMHd0MR0lZ2bXSgShASKaCIDa+X0+f6OqKXZ+MZedY5jT1Llrz8XdaaNWftUwV/TkbfCEA3jW0bSZIEYFbl7+ckCrXiQSIDqYCQivJUN1BGB5gzQHshhOBklghB+gwhb4jBV+NQGsLgFLx1hRwkI889c+88A88yQxi/vkSsGWvHGm5VvXzWmlZO1r6tE/oWzenP+702GhXvqTsERQKgYdewN4GBWnSAjoJECN5xOUJA2PponihB3TLSiohRTTUGGNoUQYAo8KYfvqlIAtdK100ePL+ZcvTcyzXDdmbhPnj3rAlrU3j4rFkK67OWuYX2ZyUCGtvuGNs2yip7AtBGoXhPWW1q0gYYvsMyUnA+i8Xs6Gd4Njx7nrUw9mI9K6xrb4btkrcCaZlAVIjt2iAbkgfpA4zjfrwJgx93iBwkI889c+88A8/CM6lvXAMx0CIMKIwWCUMFqYGYEQM7pVFNNQYY2hRBwMPmfRbvpl5cK12Xf/Jb/GbK0WPguSf1iWsgBjqEAYXVIWGpYDUwYkAMiAEx0BQGJAASADEgBsSAGBADGWJAoWco9KbYo9fRExEDYkAMdBcDEgAJgBgQA2JADIiBDDGg0DMUuoy9u4xd2Sk7MSAGmsKABEACIAbEgBgQA2IgQwwo9AyF3hR79Dp6ImJADIiB7mJAAiABEANiQAyIATGQIQYUeoZCl7F3l7ErO2UnBsRAUxiQAEgAxIAYEANiQAxkiAGFnqHQm2KPXkdPRAyIATHQXQxIACQAYkAMiAExIAYyxIBCz1DoMvbuMnZlp+zEgBhoCgMSAAmAGBADYkAMiIEMMaDQMxR6U+zR6+iJiAExIAa6iwEJgARADIgBMSAGxECGGFDoGQpdxt5dxq7slJ0YEANNYUACIAEQA2JADIgBMZAhBhR6hkJvij16HT0RMSAGxEB3MSABkACIATEgBsSAGMgQAwo9Q6HL2LvL2JWdshMDYqApDEgAJABiQAyIATEgBjLEgELPUOhNsUevoyciBsSAGOguBiQAEgAxIAbEgBgQAxliQKFnKHQZe3cZu7JTdmJADDSFAQmABEAMiAExIAbEQIYYUOgZCr0p9uh19ETEgBgQA93FgARAAiAGxIAYEANiIEMMKPQMhS5j7y5jV3bKTgyIgaYwIAGQAIgBMSAGxIAYyBADCj1DoTfFHr2OnogYEANioLsYkABIAMSAGBADYkAMZIgBhZ6h0GXs3WXsyk7ZiQEx0BQGJAASADEgBsSAGBADGWJAoWco9KbYo9fRExEDYkAMdBcDEgAJgBgQA2JADIiBDDGg0DMUuoy9u4xd2Sk7MSAGmsKABEACIAbEgBgQA2IgQwwo9AyF3hR79Dp6ImJADIiB7mJAAiABEANiQAyIATGQIQYUeoZCl7F3l7ErO2UnBsRAUxiQAEgAxIAYEANiQAxkiAGFnqHQm2KPXkdPRAyIATHQXQxIACQAYkAMiAExIAYyxIBCz1DoMvbuMnZlp+zEgBhoCgMSAAmAGBADYkAMiIEMMaDQMxR6U+zR6+iJiAExIAa6iwEJgARADIgBMSAGxECGGFDoGQpdxt5dxq7slJ0YEANNYUACIAEQA2JADIgBMZAhBhR6hkJvij16HT0RMSAGxEB3MSABkACIATEgBsSAGMgQAwo9Q6HL2LvL2JWdshMDYqApDEgAJABiQAyIATEgBjLEgELPUOhNsUevoyciBsSAGOguBiQAEgAxIAbEgBgQAxliQKFnKHQZe3cZu7JTdmJADDSFAQmABEAMiAExIAbEQIYYUOgZCr0p9uh19ETEgBgQA93FgARAAiAGxIAYEANiIEMMKPQMhS5j7y5jV3bKTgyIgaYwIAGQAIgBMSAGxIAYyBADCj1DoTfFHr2OnogYEANioLsYkABIAMSAGBADYkAMZIgBhZ6h0GXs3WXsyk7ZiQEx0BQGJAASADEgBsSAGBADGWJAoWco9KbYo9fRExEDYkAMdBcDEgAJgBgQA2JADIiBDDGg0DMUuoy9u4xd2Sk7MSAGmsKABEACIAbEgBgQA2IgQwwo9AyF3hR79Dp6ImJADIiB7mJAAiABEANiQAyIATGQIQYUeoZCl7F3l7ErO2UnBsRAUxiQAEgAxIAYEANiQAxkiAGFnqHQm2KPXkdPRAyIATHQXQxIACQAYkAMiAExIAYyxIBCz1DoMvbuMnZlp+zEgBhoCgMSAAmAGBADYkAMiIEMMaDQMxR6U+zR6+iJiAExIAa6iwEJgARADIgBMSAGxECGGFDoGQpdxt5dxq7slJ0YEANNYUACIAEQA2JADIgBMZAhBhR6hkJvij16HT0RMSAGxEB3MSABkACIATEgBsSAGMgQAwo9Q6HL2LvL2JWdshMDYqApDPz/YhNlzXxaJyoAAAAASUVORK5CYII=",
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
    Modelica(version="3.2.1"),
    WurmKomponenten(version="1.0.5"),
      TILMedia(version="3.4.0"),
      TIL3_AddOn_Cabin(version="3.1.3"),
      TILFileReader(version="3.3.2"),
      TIL(version="3.4.1")),
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
    version="1",
    conversion(noneFromVersion=""));
end TIL3_AddOn_Supermarkt_Wurm_Februar17;

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




package Testbench
  model Complete_System "Supermarket Refrigeration System CO2"
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
    Modelica.Blocks.Interfaces.RealOutput Tgc_Measurement annotation (Placement(
          transformation(extent={{152,220},{172,240}}), iconTransformation(
            extent={{146,220},{166,240}})));
    Modelica.Blocks.Interfaces.RealOutput hp_Measurement annotation (Placement(
          transformation(extent={{144,52},{164,72}}), iconTransformation(extent=
             {{144,52},{164,72}})));
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
    connect(sensor_T.sensorValue, Tgc_Measurement) annotation (Line(points={{
            -192,216.2},{-18,216.2},{-18,230},{162,230}}, color={0,0,127}));
    connect(sensor_HP.sensorValue, hp_Measurement) annotation (Line(points={{
            -235.8,178},{-44,178},{-44,62},{154,62}}, color={0,0,127}));
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
  end Complete_System;

  model Steady_State_Generator
    "Generates Optimal Steady States for System"
    parameter Real TUmgebung;

    Modelica.Blocks.Interfaces.RealOutput HPSoll
      annotation (Placement(transformation(extent={{78,-84},{98,-64}})));
    Modelica.Blocks.Interfaces.RealOutput TgcSoll
      annotation (Placement(transformation(extent={{80,44},{100,64}})));
    Modelica.Blocks.Interfaces.RealOutput TUmgebung_
      annotation (Placement(transformation(extent={{76,-10},{96,10}})));
  equation
    HPSoll=max(55, 1.62124822*TUmgebung - 400)*1e5;
    TgcSoll=TUmgebung+4;
  TUmgebung_=TUmgebung;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Steady_State_Generator;

  model Complete_System_Tester
    import Testbench;
    Testbench.Complete_System Complete_System
      annotation (Placement(transformation(extent={{-28,-2},{32,42}})));
    Modelica.Blocks.Sources.RealExpression Revolution_Fan(y=10)
      annotation (Placement(transformation(extent={{-66,28},{-46,48}})));
    Modelica.Blocks.Sources.RealExpression Ambient_Temperature(y=273.15 + 25)
      annotation (Placement(transformation(extent={{-66,12},{-46,32}})));
    Modelica.Blocks.Sources.RealExpression Area_Valve(y=4e-6)
      annotation (Placement(transformation(extent={{-68,-6},{-48,14}})));
  equation
    connect(Complete_System.unFan, Revolution_Fan.y) annotation (Line(points={{-25.36,
            39.5556},{-36,39.5556},{-36,38},{-45,38}},        color={0,0,127}));
    connect(Complete_System.uTUmgebung, Ambient_Temperature.y) annotation (Line(
          points={{-25.6,22.9333},{-37,22.9333},{-37,22},{-45,22}}, color={0,0,
            127}));
    connect(Complete_System.uVentil, Area_Valve.y) annotation (Line(points={{
            -25.6,5.33333},{-36,5.33333},{-36,4},{-47,4}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}})));
  end Complete_System_Tester;

  model Steady_State_Generator_Simulation
    "Generates the physical States of the complete System"
    import Testbench;
    Testbench.Steady_State_Generator Steady_State_Generator(TUmgebung=273.15 +
          15)
      annotation (Placement(transformation(extent={{-106,16},{-86,36}})));
    Testbench.Complete_System gascooler_Valve_System
      annotation (Placement(transformation(extent={{-8,-2},{42,34}})));
    TIL.OtherComponents.Controllers.PIController Fan(
      controllerType="PI",
      invertFeedback=false,
      Ti=400,
      use_activeInput=true,
      yInitial=10,
      yMax=20,
      yMin=0,
      k=-0.4) annotation (Placement(transformation(
          extent={{-6,6},{6,-6}},
          rotation=0,
          origin={-50,46})));
    TIL.OtherComponents.Controllers.PIController Valve(
      controllerType="PI",
      invertFeedback=false,
      use_activeInput=true,
      k=-2e-13,
      Ti=60,
      yMax=1e-2,
      yMin=1e-12,
      yInitial=4e-6) annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={-54,4})));
    Modelica.Blocks.Sources.BooleanStep Activator(startValue=true, startTime=
          4000)
      annotation (Placement(transformation(extent={{-104,58},{-84,78}})));
  equation
    connect(Fan.y, gascooler_Valve_System.unFan) annotation (Line(points={{
            -43.6,46},{-43.6,32},{-5.8,32}}, color={0,0,127}));
    connect(Valve.y, gascooler_Valve_System.uVentil)
      annotation (Line(points={{-47.6,4},{-30,4},{-6,4}}, color={0,0,127}));
    connect(Steady_State_Generator.HPSoll, Valve.u_s) annotation (Line(points={
            {-87.2,18.6},{-74,18.6},{-74,4},{-59.6,4}}, color={0,0,127}));
    connect(Steady_State_Generator.TgcSoll, Fan.u_s) annotation (Line(points={{
            -87,31.4},{-72,31.4},{-72,46},{-55.6,46}}, color={0,0,127}));
    connect(Steady_State_Generator.TUmgebung_, gascooler_Valve_System.uTUmgebung)
      annotation (Line(points={{-87.4,26},{-46,26},{-46,18.4},{-6,18.4}}, color=
           {0,0,127}));
    connect(Fan.activeInput, Activator.y) annotation (Line(points={{-55.6,49.8},
            {-58,49.8},{-58,68},{-83,68}}, color={255,0,255}));
    connect(gascooler_Valve_System.Tgc_Measurement, Fan.u_m) annotation (Line(
          points={{41.6,27},{41.6,50},{-50,50},{-50,51.8}}, color={0,0,127}));
    connect(gascooler_Valve_System.hp_Measurement, Valve.u_m) annotation (Line(
          points={{41.4,10.2},{-23.6,10.2},{-23.6,-1.8},{-54,-1.8}}, color={0,0,
            127}));
    connect(Valve.activeInput, Activator.y) annotation (Line(points={{-59.6,0.2},
            {-59.6,34.1},{-83,34.1},{-83,68}}, color={255,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=10000, __Dymola_NumberOfIntervals=10000));
  end Steady_State_Generator_Simulation;

  model Gascooler_Valve_System "Simple Model derived from Supermarket Model"
   parameter Real pHochdruckStart=100e5;
   parameter Real EnthalpyGasCoolerInlet = 520e3;
   parameter Real EnthalpyGasCoolerOutletStart = 300e3;
   parameter Real pMitteldruckStart = 35e5;
   parameter Real MassflowGasCoolerInlet = -0.3;
   parameter Real FanRevolution = 10;

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
      portB(p(start=pMitteldruckStart)),
      vleFluidA(h(start=EnthalpyGasCoolerOutletStart)))   annotation (Placement(transformation(
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
    TIL.VLEFluidComponents.Sensors.StatePoint Gascooler_Entry(stateViewerIndex=1)
      annotation (Placement(transformation(extent={{26,206},{34,214}})));
    TIL.VLEFluidComponents.Sensors.StatePoint Gascooler_Outlet(stateViewerIndex=2)
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
    Modelica.Blocks.Continuous.FirstOrder firstOrder(y_start=10, T=1e-3)
      annotation (Placement(transformation(extent={{-156,278},{-148,286}})));
    TIL.HeatExchangers.FinAndTube.GasVLEFluid.CrossFlowHX Gascooler(
      gasCellFlowType="flow B-A",
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
      hInitialVLEFluid_Cell1=EnthalpyGasCoolerOutletStart,
      hInitialVLEFluid_CellN=EnthalpyGasCoolerInlet,
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
      annotation (Placement(transformation(extent={{-340,240},{-300,280}}),
          iconTransformation(extent={{-340,240},{-300,280}})));
    Modelica.Blocks.Interfaces.RealInput uVentil
      annotation (Placement(transformation(extent={{-340,-20},{-300,20}}),
          iconTransformation(extent={{-340,-20},{-300,20}})));
    Modelica.Blocks.Interfaces.RealInput uTUmgebung
      annotation (Placement(transformation(extent={{-340,120},{-300,160}}),
          iconTransformation(extent={{-340,120},{-300,160}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder3(
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=300,
      T=0.1)
      annotation (Placement(transformation(extent={{-162,246},{-154,254}})));
    TIL.VLEFluidComponents.Boundaries.BoundaryOverdetermined
      boundaryOverdetermined(
      hFixed=EnthalpyGasCoolerInlet,
      m_flowFixed = MassflowGasCoolerInlet,
      pFixed(displayUnit="Pa") = pHochdruckStart)
      annotation (Placement(transformation(extent={{92,192},{100,212}})));
    TIL.VLEFluidComponents.Boundaries.BoundaryUnderdetermined
      boundaryUnderdetermined annotation (Placement(transformation(
          extent={{-4,-10},{4,10}},
          rotation=90,
          origin={-258,94})));
    TIL.VLEFluidComponents.Sensors.StatePoint Valve_Outlet(stateViewerIndex=3)
      annotation (Placement(transformation(extent={{-234,120},{-226,128}})));
    Modelica.Blocks.Interfaces.RealOutput Tgc_Measurement annotation (Placement(
          transformation(extent={{140,250},{160,270}}), iconTransformation(extent=
             {{140,250},{160,270}})));
    Modelica.Blocks.Interfaces.RealOutput hp_Measurement annotation (Placement(
          transformation(extent={{140,-10},{160,10}}), iconTransformation(extent={
              {140,-10},{160,10}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder1(
                                                     y_start=10,
      k=1,
      T=1e-3)
      annotation (Placement(transformation(extent={{-294,84},{-286,92}})));
  equation

    connect(sensor_HP.port,HPValve. portA) annotation (Line(
        points={{-242,178},{-258,178},{-258,156}},
        color={153,204,0},
        thickness=0.5,
        smooth=Smooth.None));
    connect(Gascooler_Outlet.sensorPort, HPValve.portA) annotation (Line(
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
    connect(Gascooler_Entry.sensorPort, pressureState_2.portB) annotation (Line(
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
        points={{-94,260},{-102,260},{-102,282},{-147.6,282}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(rotatoryBoundary1.n_in, firstOrder.y) annotation (Line(
        points={{-94,230},{-104,230},{-104,282},{-147.6,282}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(firstOrder.u, unFan) annotation (Line(
        points={{-156.8,282},{-220,282},{-220,260},{-320,260}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(uTUmgebung, firstOrder3.u) annotation (Line(points={{-320,140},{-320,250},
            {-162.8,250}},            color={0,0,127}));
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
    connect(Valve_Outlet.sensorPort, HPValve.portB) annotation (Line(
        points={{-230,120},{-248,120},{-258,120},{-258,140}},
        color={153,204,0},
        thickness=0.5));
    connect(sensor_T.sensorValue, Tgc_Measurement) annotation (Line(points={{-192,
            216.2},{-190,216.2},{-190,260},{150,260}}, color={0,0,127}));
    connect(sensor_HP.sensorValue, hp_Measurement) annotation (Line(points={{-235.8,
            178},{150,178},{150,0}}, color={0,0,127}));
    connect(Tgc_Measurement, Tgc_Measurement) annotation (Line(points={{150,260},{
            156,260},{156,260},{150,260}}, color={0,0,127}));
    connect(hp_Measurement, hp_Measurement) annotation (Line(points={{150,0},{154,
            0},{154,-12},{154,0},{150,0}}, color={0,0,127}));
    connect(HPValve.effectiveFlowArea_in, firstOrder1.y) annotation (Line(
          points={{-263,148},{-268,148},{-268,88},{-285.6,88}}, color={0,0,127}));
    connect(uVentil, firstOrder1.u) annotation (Line(points={{-320,0},{-300,0},
            {-300,88},{-294.8,88}}, color={0,0,127}));
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
  end Gascooler_Valve_System;

  model Gascooler_Valve_System_Tester_Constant_Valve_1
                                                      "Running"
    Testbench.Gascooler_Valve_System Gascooler_Valve_System(
      pHochdruckStart=77e5,
      HPValve(effectiveFlowAreaTypical=4e-6),
      boundaryUnderdetermined(port(m_flow(start=0.3))),
      pMitteldruckStart=35e5,
      EnthalpyGasCoolerInlet=520e3,
      MassflowGasCoolerInlet=-0.3)
      annotation (Placement(transformation(extent={{-14,4},{36,40}})));
    Modelica.Blocks.Sources.RealExpression Ambient_Temperature(y=273.15 + 25)
      annotation (Placement(transformation(extent={{-72,14},{-52,34}})));
    Modelica.Blocks.Sources.Ramp Area_Valve(
      offset=1e-4,
      height=-1e-4 + 4e-6,
      duration=8,
      startTime=2)
      annotation (Placement(transformation(extent={{-42,4},{-30,16}})));
    Modelica.Blocks.Sources.Step Revolution_Fan(
      height=10,
      offset=10,
      startTime=100)
      annotation (Placement(transformation(extent={{-42,30},{-30,42}})));
  equation
    connect(Gascooler_Valve_System.uTUmgebung, Ambient_Temperature.y)
      annotation (Line(points={{-12,24},{-37,24},{-51,24}}, color={0,0,127}));
    connect(Area_Valve.y, Gascooler_Valve_System.uVentil) annotation (Line(
          points={{-29.4,10},{-29.4,10},{-12,10}}, color={0,0,127}));
    connect(Revolution_Fan.y, Gascooler_Valve_System.unFan) annotation (Line(
          points={{-29.4,36},{-29.4,36},{-12,36}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}}), graphics={Text(
            extent={{-14,62},{40,40}},
            lineColor={28,108,200},
            textString="Objective:
Constant Area in Valve -> Jump on revolution

Bernoulli-Gleichung im Ventil --> da geht die Dichte ein",
            horizontalAlignment=TextAlignment.Left)}),
      experiment(StopTime=200, __Dymola_NumberOfIntervals=10000));
  end Gascooler_Valve_System_Tester_Constant_Valve_1;

  model Gascooler_Valve_System_Tester_Constant_Valve_2 "Alternate Steady State"
    Testbench.Gascooler_Valve_System Gascooler_Valve_System(
      HPValve(effectiveFlowAreaTypical=4e-6),
      boundaryUnderdetermined(port(m_flow(start=0.3))),
      pHochdruckStart=75e5,
      EnthalpyGasCoolerInlet=515e3,
      EnthalpyGasCoolerOutletStart=263e3,
      pMitteldruckStart=30e5,
      MassflowGasCoolerInlet=-0.28,
      FanRevolution=7.4)
      annotation (Placement(transformation(extent={{-14,4},{36,40}})));
    Modelica.Blocks.Sources.RealExpression Ambient_Temperature(y=273.15 + 20)
      annotation (Placement(transformation(extent={{-72,14},{-52,34}})));
    Modelica.Blocks.Sources.Ramp Area_Valve(
      offset=1e-4,
      duration=8,
      startTime=2,
      height=-1e-4 + 3.2e-6)
      annotation (Placement(transformation(extent={{-42,4},{-30,16}})));
    Modelica.Blocks.Sources.Step Revolution_Fan(
      height=10,
      startTime=100,
      offset=7.4)
      annotation (Placement(transformation(extent={{-42,30},{-30,42}})));
  equation
    connect(Gascooler_Valve_System.uTUmgebung, Ambient_Temperature.y)
      annotation (Line(points={{-12,24},{-37,24},{-51,24}}, color={0,0,127}));
    connect(Area_Valve.y, Gascooler_Valve_System.uVentil) annotation (Line(
          points={{-29.4,10},{-29.4,10},{-12,10}}, color={0,0,127}));
    connect(Revolution_Fan.y, Gascooler_Valve_System.unFan) annotation (Line(
          points={{-29.4,36},{-29.4,36},{-12,36}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}}), graphics={Text(
            extent={{-14,62},{40,40}},
            lineColor={28,108,200},
            textString="Objective:
Constant Area in Valve -> Jump on revolution

Bernoulli-Gleichung im Ventil --> da geht die Dichte ein",
            horizontalAlignment=TextAlignment.Left)}),
      experiment(StopTime=200, __Dymola_NumberOfIntervals=10000));
  end Gascooler_Valve_System_Tester_Constant_Valve_2;

  model Gascooler_Valve_System_Tester_Constant_Fan_1 "Running"
    Testbench.Gascooler_Valve_System Gascooler_Valve_System(
      pHochdruckStart=77e5,
      HPValve(effectiveFlowAreaTypical=4e-6),
      boundaryUnderdetermined(port(m_flow(start=0.3))),
      pMitteldruckStart=35e5,
      EnthalpyGasCoolerInlet=520e3,
      MassflowGasCoolerInlet=-0.3)
      annotation (Placement(transformation(extent={{-14,4},{36,40}})));
    Modelica.Blocks.Sources.RealExpression Ambient_Temperature(y=273.15 + 25)
      annotation (Placement(transformation(extent={{-78,18},{-58,38}})));
    Modelica.Blocks.Sources.Ramp Area_Valve(
      offset=1e-4,
      height=-1e-4 + 4e-6,
      duration=8,
      startTime=2)
      annotation (Placement(transformation(extent={{-76,6},{-64,18}})));
    Modelica.Blocks.Sources.Step Revolution_Fan(
      offset=10,
      startTime=100,
      height=0)
      annotation (Placement(transformation(extent={{-42,30},{-30,42}})));
    Modelica.Blocks.Sources.Step Revolution_Fan1(
      startTime=100,
      height=1,
      offset=1)
      annotation (Placement(transformation(extent={{-76,-14},{-64,-2}})));
    Modelica.Blocks.Math.Product product
      annotation (Placement(transformation(extent={{-42,6},{-32,16}})));
  equation
    connect(Gascooler_Valve_System.uTUmgebung, Ambient_Temperature.y)
      annotation (Line(points={{-12,24},{-30,24},{-30,28},{-57,28}},
                                                            color={0,0,127}));
    connect(Revolution_Fan.y, Gascooler_Valve_System.unFan) annotation (Line(
          points={{-29.4,36},{-29.4,36},{-12,36}}, color={0,0,127}));
    connect(Area_Valve.y, product.u1) annotation (Line(points={{-63.4,12},{-54,
            12},{-54,14},{-43,14}}, color={0,0,127}));
    connect(Revolution_Fan1.y, product.u2) annotation (Line(points={{-63.4,-8},
            {-54,-8},{-54,8},{-43,8}}, color={0,0,127}));
    connect(Gascooler_Valve_System.uVentil, product.y) annotation (Line(points=
            {{-12,10},{-22,10},{-22,11},{-31.5,11}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}}), graphics={Text(
            extent={{-14,62},{40,40}},
            lineColor={28,108,200},
            textString="Objective:
Constant Area in Valve -> Jump on revolution

Bernoulli-Gleichung im Ventil --> da geht die Dichte ein",
            horizontalAlignment=TextAlignment.Left)}),
      experiment(StopTime=200, __Dymola_NumberOfIntervals=10000));
  end Gascooler_Valve_System_Tester_Constant_Fan_1;

  model Gascooler_Valve_System_Tester_Constant_Fan_2 "Alternate Steady State"
    Testbench.Gascooler_Valve_System Gascooler_Valve_System(
      HPValve(effectiveFlowAreaTypical=4e-6),
      boundaryUnderdetermined(port(m_flow(start=0.3))),
      pHochdruckStart=75e5,
      EnthalpyGasCoolerInlet=515e3,
      EnthalpyGasCoolerOutletStart=263e3,
      pMitteldruckStart=30e5,
      MassflowGasCoolerInlet=-0.28,
      FanRevolution=7.4)
      annotation (Placement(transformation(extent={{-14,4},{36,40}})));
    Modelica.Blocks.Sources.RealExpression Ambient_Temperature(y=273.15 + 20)
      annotation (Placement(transformation(extent={{-46,16},{-34,32}})));
    Modelica.Blocks.Sources.Ramp Area_Valve(
      offset=1e-4,
      duration=8,
      startTime=2,
      height=-1e-4 + 3.2e-6)
      annotation (Placement(transformation(extent={{-68,4},{-56,16}})));
    Modelica.Blocks.Sources.Step Revolution_Fan(
      startTime=100,
      height=0,
      offset=7.4)
      annotation (Placement(transformation(extent={{-68,30},{-56,42}})));
    Modelica.Blocks.Sources.Step Revolution_Fan1(
      startTime=100,
      height=1,
      offset=1)
      annotation (Placement(transformation(extent={{-68,-16},{-56,-4}})));
    Modelica.Blocks.Math.Product product
      annotation (Placement(transformation(extent={{-40,4},{-34,10}})));
  equation
    connect(Gascooler_Valve_System.uTUmgebung, Ambient_Temperature.y)
      annotation (Line(points={{-12,24},{-33.4,24}},        color={0,0,127}));
    connect(Revolution_Fan.y, Gascooler_Valve_System.unFan) annotation (Line(
          points={{-55.4,36},{-55.4,36},{-12,36}}, color={0,0,127}));
    connect(Gascooler_Valve_System.uVentil, product.y) annotation (Line(points=
            {{-12,10},{-22,10},{-22,7},{-33.7,7}}, color={0,0,127}));
    connect(Area_Valve.y, product.u1) annotation (Line(points={{-55.4,10},{-48,
            10},{-48,8.8},{-40.6,8.8}}, color={0,0,127}));
    connect(Revolution_Fan1.y, product.u2) annotation (Line(points={{-55.4,-10},
            {-48,-10},{-48,5.2},{-40.6,5.2}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
              -20},{60,60}})), Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-80,-20},{60,60}}), graphics={Text(
            extent={{-14,62},{40,40}},
            lineColor={28,108,200},
            textString="Objective:
Constant Area in Valve -> Jump on revolution

Bernoulli-Gleichung im Ventil --> da geht die Dichte ein",
            horizontalAlignment=TextAlignment.Left)}),
      experiment(StopTime=200, __Dymola_NumberOfIntervals=10000));
  end Gascooler_Valve_System_Tester_Constant_Fan_2;
  annotation (uses(
      TIL(version="3.4.1"),
      Modelica(version="3.2.1"),
      TILMedia(version="3.4.1"),
      TIL3_AddOn_Supermarkt_Wurm_Februar17(version="1")));
end Testbench;
