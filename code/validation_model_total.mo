
package Modelica "Modelica Standard Library - Version 3.2.3"
extends Modelica.Icons.Package;

  package Blocks "Library of basic input/output control blocks (continuous, discrete, logical, table blocks)"
  import SI = Modelica.SIunits;
  extends Modelica.Icons.Package;

    package Continuous "Library of continuous control blocks with internal states"
      import Modelica.Blocks.Interfaces;
      import Modelica.SIunits;
      extends Modelica.Icons.Package;

      block Integrator "Output the integral of the input signal with optional reset"
        import Modelica.Blocks.Types.Init;
        parameter Real k(unit="1")=1 "Integrator gain";
        parameter Boolean use_reset = false "=true, if reset port enabled"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_set = false "=true, if set port enabled and used as reinitialization value when reset"
          annotation(Dialog(enable=use_reset), Evaluate=true, HideResult=true, choices(checkBox=true));

        /* InitialState is the default, because it was the default in Modelica 2.2
     and therefore this setting is backward compatible
  */
        parameter Modelica.Blocks.Types.Init initType=Modelica.Blocks.Types.Init.InitialState
          "Type of initialization (1: no init, 2: steady state, 3,4: initial output)" annotation(Evaluate=true,
            Dialog(group="Initialization"));
        parameter Real y_start=0 "Initial or guess value of output (= state)"
          annotation (Dialog(group="Initialization"));
        extends Interfaces.SISO(y(start=y_start));
        Modelica.Blocks.Interfaces.BooleanInput reset if use_reset "Optional connector of reset signal" annotation(Placement(
          transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={60,-120})));
        Modelica.Blocks.Interfaces.RealInput set if use_reset and use_set "Optional connector of set signal" annotation(Placement(
          transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,120})));
      protected
        Modelica.Blocks.Interfaces.BooleanOutput local_reset annotation(HideResult=true);
        Modelica.Blocks.Interfaces.RealOutput local_set annotation(HideResult=true);

      initial equation
        if initType == Init.SteadyState then
           der(y) = 0;
        elseif initType == Init.InitialState or
               initType == Init.InitialOutput then
          y = y_start;
        end if;
      equation
        if use_reset then
          connect(reset, local_reset);
          if use_set then
            connect(set, local_set);
          else
            local_set = y_start;
          end if;
          when local_reset then
            reinit(y, local_set);
          end when;
        else
          local_reset = false;
          local_set = 0;
        end if;
        der(y) = k*u;
        annotation (
          Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as
<em>integral</em> of the input <strong>u</strong> multiplied with
the gain <em>k</em>:
</p>
<pre>
         k
     y = - u
         s
</pre>

<p>
It might be difficult to initialize the integrator in steady state.
This is discussed in the description of package
<a href=\"modelica://Modelica.Blocks.Continuous#info\">Continuous</a>.
</p>

<p>
If the <em>reset</em> port is enabled, then the output <strong>y</strong> is reset to <em>set</em>
or to <em>y_start</em> (if the <em>set</em> port is not enabled), whenever the <em>reset</em>
port has a rising edge.
</p>
</html>"),     Icon(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
                Line(
                  points={{-80.0,78.0},{-80.0,-90.0}},
                  color={192,192,192}),
                Polygon(
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid,
                  points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
                Line(
                  points={{-90.0,-80.0},{82.0,-80.0}},
                  color={192,192,192}),
                Polygon(
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid,
                  points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
                Text(
                  lineColor={192,192,192},
                  extent={{0.0,-70.0},{60.0,-10.0}},
                  textString="I"),
                Text(
                  extent={{-150.0,-150.0},{150.0,-110.0}},
                  textString="k=%k"),
                Line(
                  points=DynamicSelect({{-80.0,-80.0},{80.0,80.0}}, if use_reset then {{-80.0,-80.0},{60.0,60.0},{60.0,-80.0},{80.0,-60.0}} else {{-80.0,-80.0},{80.0,80.0}}),
                  color={0,0,127}),
                Line(
                  visible=use_reset,
                  points={{60,-100},{60,-80}},
                  color={255,0,255},
                  pattern=LinePattern.Dot),
                Text(
                  visible=use_reset,
                  extent={{-28,-62},{94,-86}},
                  textString="reset")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,255}),
              Line(points={{-100,0},{-60,0}}, color={0,0,255}),
              Line(points={{60,0},{100,0}}, color={0,0,255}),
              Text(
                extent={{-36,60},{32,2}},
                textString="k"),
              Text(
                extent={{-32,0},{36,-58}},
                textString="s"),
              Line(points={{-46,0},{46,0}})}));
      end Integrator;

      block Derivative "Approximated derivative block"
        import Modelica.Blocks.Types.Init;
        parameter Real k(unit="1")=1 "Gains";
        parameter SIunits.Time T(min=Modelica.Constants.small) = 0.01
          "Time constants (T>0 required; T=0 is ideal derivative block)";
        parameter Modelica.Blocks.Types.Init initType=Modelica.Blocks.Types.Init.NoInit
          "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
                                                                                          annotation(Evaluate=true,
            Dialog(group="Initialization"));
        parameter Real x_start=0 "Initial or guess value of state"
          annotation (Dialog(group="Initialization"));
        parameter Real y_start=0 "Initial value of output (= state)"
          annotation(Dialog(enable=initType == Init.InitialOutput, group=
                "Initialization"));
        extends Interfaces.SISO;

        output Real x(start=x_start) "State of block";

      protected
        parameter Boolean zeroGain = abs(k) < Modelica.Constants.eps;
      initial equation
        if initType == Init.SteadyState then
          der(x) = 0;
        elseif initType == Init.InitialState then
          x = x_start;
        elseif initType == Init.InitialOutput then
          if zeroGain then
             x = u;
          else
             y = y_start;
          end if;
        end if;
      equation
        der(x) = if zeroGain then 0 else (u - x)/T;
        y = if zeroGain then 0 else (k/T)*(u - x);
        annotation (
          Documentation(info="<html>
<p>
This blocks defines the transfer function between the
input u and the output y
as <em>approximated derivative</em>:
</p>
<pre>
             k * s
     y = ------------ * u
            T * s + 1
</pre>
<p>
If you would like to be able to change easily between different
transfer functions (FirstOrder, SecondOrder, ... ) by changing
parameters, use the general block <strong>TransferFunction</strong> instead
and model a derivative block with parameters<br>
b = {k,0}, a = {T, 1}.
</p>

<p>
If k=0, the block reduces to y=0.
</p>
</html>"),     Icon(
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
        Line(origin = {-24.667,-27.333},
          points = {{-55.333,87.333},{-19.333,-40.667},{86.667,-52.667}},
          color = {0,0,127},
          smooth = Smooth.Bezier),
        Text(lineColor={192,192,192},
          extent={{-30.0,14.0},{86.0,60.0}},
          textString="DT1"),
        Text(extent={{-150.0,-150.0},{150.0,-110.0}},
          textString="k=%k")}),
          Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Text(
                extent={{-54,52},{50,10}},
                textString="k s"),
              Text(
                extent={{-54,-6},{52,-52}},
                textString="T s + 1"),
              Line(points={{-50,0},{50,0}}),
              Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,255}),
              Line(points={{-100,0},{-60,0}}, color={0,0,255}),
              Line(points={{60,0},{100,0}}, color={0,0,255})}));
      end Derivative;

      block LimPID
        "P, PI, PD, and PID controller with limited output, anti-windup compensation, setpoint weighting and optional feed-forward"
        import Modelica.Blocks.Types.InitPID;
        import Modelica.Blocks.Types.Init;
        import Modelica.Blocks.Types.SimpleController;
        extends Modelica.Blocks.Interfaces.SVcontrol;
        output Real controlError = u_s - u_m
          "Control error (set point - measurement)";
        parameter .Modelica.Blocks.Types.SimpleController controllerType=
               .Modelica.Blocks.Types.SimpleController.PID "Type of controller";
        parameter Real k(min=0, unit="1") = 1 "Gain of controller";
        parameter Modelica.SIunits.Time Ti(min=Modelica.Constants.small)=0.5
          "Time constant of Integrator block" annotation (Dialog(enable=
                controllerType == .Modelica.Blocks.Types.SimpleController.PI or
                controllerType == .Modelica.Blocks.Types.SimpleController.PID));
        parameter Modelica.SIunits.Time Td(min=0)=0.1
          "Time constant of Derivative block" annotation (Dialog(enable=
                controllerType == .Modelica.Blocks.Types.SimpleController.PD or
                controllerType == .Modelica.Blocks.Types.SimpleController.PID));
        parameter Real yMax(start=1) "Upper limit of output";
        parameter Real yMin=-yMax "Lower limit of output";
        parameter Real wp(min=0) = 1
          "Set-point weight for Proportional block (0..1)";
        parameter Real wd(min=0) = 0 "Set-point weight for Derivative block (0..1)"
           annotation(Dialog(enable=controllerType==.Modelica.Blocks.Types.SimpleController.PD or
                                      controllerType==.Modelica.Blocks.Types.SimpleController.PID));
        parameter Real Ni(min=100*Modelica.Constants.eps) = 0.9
          "Ni*Ti is time constant of anti-windup compensation"
           annotation(Dialog(enable=controllerType==.Modelica.Blocks.Types.SimpleController.PI or
                                    controllerType==.Modelica.Blocks.Types.SimpleController.PID));
        parameter Real Nd(min=100*Modelica.Constants.eps) = 10
          "The higher Nd, the more ideal the derivative block"
           annotation(Dialog(enable=controllerType==.Modelica.Blocks.Types.SimpleController.PD or
                                      controllerType==.Modelica.Blocks.Types.SimpleController.PID));
        parameter Boolean withFeedForward=false "Use feed-forward input?"
          annotation(Evaluate=true, choices(checkBox=true));
        parameter Real kFF=1 "Gain of feed-forward input"
          annotation(Dialog(enable=withFeedForward));
        parameter .Modelica.Blocks.Types.InitPID initType= .Modelica.Blocks.Types.InitPID.DoNotUse_InitialIntegratorState
          "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
          annotation(Evaluate=true, Dialog(group="Initialization"));
        parameter Real xi_start=0
          "Initial or guess value for integrator output (= integrator state)"
          annotation (Dialog(group="Initialization",
                      enable=controllerType==.Modelica.Blocks.Types.SimpleController.PI or
                             controllerType==.Modelica.Blocks.Types.SimpleController.PID));
        parameter Real xd_start=0
          "Initial or guess value for state of derivative block"
          annotation (Dialog(group="Initialization",
                               enable=controllerType==.Modelica.Blocks.Types.SimpleController.PD or
                                      controllerType==.Modelica.Blocks.Types.SimpleController.PID));
        parameter Real y_start=0 "Initial value of output"
          annotation(Dialog(enable=initType == .Modelica.Blocks.Types.InitPID.InitialOutput, group=
                "Initialization"));
        parameter Modelica.Blocks.Types.LimiterHomotopy homotopyType = Modelica.Blocks.Types.LimiterHomotopy.Linear
          "Simplified model for homotopy-based initialization"
          annotation (Evaluate=true, Dialog(group="Initialization"));
        parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
          annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
        parameter Boolean limitsAtInit=true
          "Has no longer an effect and is only kept for backwards compatibility (the implementation uses now the homotopy operator)"
          annotation (Dialog(tab="Dummy"),Evaluate=true, choices(checkBox=true));
        constant Modelica.SIunits.Time unitTime=1 annotation (HideResult=true);
        Modelica.Blocks.Interfaces.RealInput u_ff if withFeedForward
          "Optional connector of feed-forward input signal"
         annotation (Placement(
              transformation(
              origin={60,-120},
              extent={{20,-20},{-20,20}},
              rotation=270)));
        Modelica.Blocks.Math.Add addP(k1=wp, k2=-1)
          annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
        Modelica.Blocks.Math.Add addD(k1=wd, k2=-1) if with_D
          annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
        Modelica.Blocks.Math.Gain P(k=1)
          annotation (Placement(transformation(extent={{-50,40},{-30,60}})));
        Modelica.Blocks.Continuous.Integrator I(
          k=unitTime/Ti,
          y_start=xi_start,
          initType=if initType == InitPID.SteadyState then Init.SteadyState else if
              initType == InitPID.InitialState or initType == InitPID.DoNotUse_InitialIntegratorState
               then Init.InitialState else Init.NoInit) if with_I
          annotation (Placement(transformation(extent={{-50,-60},{-30,-40}})));
        Modelica.Blocks.Continuous.Derivative D(
          k=Td/unitTime,
          T=max([Td/Nd,1.e-14]),
          x_start=xd_start,
          initType=if initType == InitPID.SteadyState or initType == InitPID.InitialOutput
               then Init.SteadyState else if initType == InitPID.InitialState then
              Init.InitialState else Init.NoInit) if with_D
          annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
        Modelica.Blocks.Math.Gain gainPID(k=k)
          annotation (Placement(transformation(extent={{20,-10},{40,10}})));
        Modelica.Blocks.Math.Add3 addPID
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Math.Add3 addI(k2=-1) if with_I
          annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
        Modelica.Blocks.Math.Add addSat(k1=+1, k2=-1) if with_I annotation (Placement(
              transformation(
              origin={80,-50},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Blocks.Math.Gain gainTrack(k=1/(k*Ni)) if with_I
          annotation (Placement(transformation(extent={{0,-80},{-20,-60}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(
          uMax=yMax,
          uMin=yMin,
          strict=strict,
          limitsAtInit=limitsAtInit,
          homotopyType=homotopyType)
          annotation (Placement(transformation(extent={{70,-10},{90,10}})));
      protected
        parameter Boolean with_I = controllerType==SimpleController.PI or
                                   controllerType==SimpleController.PID annotation(Evaluate=true, HideResult=true);
        parameter Boolean with_D = controllerType==SimpleController.PD or
                                   controllerType==SimpleController.PID annotation(Evaluate=true, HideResult=true);
      public
        Modelica.Blocks.Sources.Constant Dzero(k=0) if not with_D
          annotation (Placement(transformation(extent={{-40,20},{-30,30}})));
        Modelica.Blocks.Sources.Constant Izero(k=0) if not with_I
          annotation (Placement(transformation(extent={{0,-55},{-10,-45}})));
        Modelica.Blocks.Sources.Constant FFzero(k=0) if not withFeedForward
          annotation (Placement(transformation(extent={{30,-35},{40,-25}})));
        Modelica.Blocks.Math.Add addFF(k1=1, k2=kFF)
          annotation (Placement(transformation(extent={{48,-6},{60,6}})));
      initial equation
        if initType==InitPID.InitialOutput then
          gainPID.y = y_start;
        end if;
      equation
        if initType == InitPID.InitialOutput and (y_start < yMin or y_start > yMax) then
            Modelica.Utilities.Streams.error("LimPID: Start value y_start (=" + String(y_start) +
               ") is outside of the limits of yMin (=" + String(yMin) +") and yMax (=" + String(yMax) + ")");
        end if;

        connect(u_s, addP.u1) annotation (Line(points={{-120,0},{-96,0},{-96,56},{
                -82,56}}, color={0,0,127}));
        connect(u_s, addD.u1) annotation (Line(points={{-120,0},{-96,0},{-96,6},{
                -82,6}}, color={0,0,127}));
        connect(u_s, addI.u1) annotation (Line(points={{-120,0},{-96,0},{-96,-42},{
                -82,-42}}, color={0,0,127}));
        connect(addP.y, P.u) annotation (Line(points={{-59,50},{-52,50}}, color={0,
                0,127}));
        connect(addD.y, D.u)
          annotation (Line(points={{-59,0},{-52,0}}, color={0,0,127}));
        connect(addI.y, I.u) annotation (Line(points={{-59,-50},{-52,-50}}, color={
                0,0,127}));
        connect(P.y, addPID.u1) annotation (Line(points={{-29,50},{-20,50},{-20,8},{-12,
                8}},     color={0,0,127}));
        connect(D.y, addPID.u2)
          annotation (Line(points={{-29,0},{-12,0}},color={0,0,127}));
        connect(I.y, addPID.u3) annotation (Line(points={{-29,-50},{-20,-50},{-20,-8},
                {-12,-8}},    color={0,0,127}));
        connect(limiter.y, addSat.u1) annotation (Line(points={{91,0},{94,0},{94,
                -20},{86,-20},{86,-38}}, color={0,0,127}));
        connect(limiter.y, y)
          annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
        connect(addSat.y, gainTrack.u) annotation (Line(points={{80,-61},{80,-70},{2,-70}},
                          color={0,0,127}));
        connect(gainTrack.y, addI.u3) annotation (Line(points={{-21,-70},{-88,-70},{-88,
                -58},{-82,-58}},     color={0,0,127}));
        connect(u_m, addP.u2) annotation (Line(
            points={{0,-120},{0,-92},{-92,-92},{-92,44},{-82,44}},
            color={0,0,127},
            thickness=0.5));
        connect(u_m, addD.u2) annotation (Line(
            points={{0,-120},{0,-92},{-92,-92},{-92,-6},{-82,-6}},
            color={0,0,127},
            thickness=0.5));
        connect(u_m, addI.u2) annotation (Line(
            points={{0,-120},{0,-92},{-92,-92},{-92,-50},{-82,-50}},
            color={0,0,127},
            thickness=0.5));
        connect(Dzero.y, addPID.u2) annotation (Line(points={{-29.5,25},{-24,25},{-24,
                0},{-12,0}},    color={0,0,127}));
        connect(Izero.y, addPID.u3) annotation (Line(points={{-10.5,-50},{-20,-50},{-20,
                -8},{-12,-8}},    color={0,0,127}));
        connect(addPID.y, gainPID.u)
          annotation (Line(points={{11,0},{18,0}}, color={0,0,127}));
        connect(addFF.y, limiter.u)
          annotation (Line(points={{60.6,0},{68,0}}, color={0,0,127}));
        connect(gainPID.y, addFF.u1) annotation (Line(points={{41,0},{44,0},{44,3.6},
                {46.8,3.6}},color={0,0,127}));
        connect(FFzero.y, addFF.u2) annotation (Line(points={{40.5,-30},{44,-30},{44,
                -3.6},{46.8,-3.6}},
                              color={0,0,127}));
        connect(addFF.u2, u_ff) annotation (Line(points={{46.8,-3.6},{44,-3.6},{44,
                -92},{60,-92},{60,-120}},
                                     color={0,0,127}));
        connect(addFF.y, addSat.u2) annotation (Line(points={{60.6,0},{64,0},{64,-20},
                {74,-20},{74,-38}}, color={0,0,127}));
        annotation (defaultComponentName="PID",
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
              Polygon(
                points={{-80,90},{-88,68},{-72,68},{-80,90}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
              Polygon(
                points={{90,-80},{68,-72},{68,-88},{90,-80}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,-80},{-80,-20},{30,60},{80,60}}, color={0,0,127}),
              Text(
                extent={{-20,-20},{80,-60}},
                lineColor={192,192,192},
                textString="%controllerType"),
              Line(
                visible=strict,
                points={{30,60},{81,60}},
                color={255,0,0})}),
          Diagram(graphics={Text(
                  extent={{79,-112},{129,-102}},
                  lineColor={0,0,255},
                textString=" (feed-forward)")}),
          Documentation(info="<html>
<p>
Via parameter <strong>controllerType</strong> either <strong>P</strong>, <strong>PI</strong>, <strong>PD</strong>,
or <strong>PID</strong> can be selected. If, e.g., PI is selected, all components belonging to the
D-part are removed from the block (via conditional declarations).
The example model
<a href=\"modelica://Modelica.Blocks.Examples.PID_Controller\">Modelica.Blocks.Examples.PID_Controller</a>
demonstrates the usage of this controller.
Several practical aspects of PID controller design are incorporated
according to chapter 3 of the book:
</p>

<dl>
<dt>&Aring;str&ouml;m K.J., and H&auml;gglund T.:</dt>
<dd> <strong>PID Controllers: Theory, Design, and Tuning</strong>.
     Instrument Society of America, 2nd edition, 1995.
</dd>
</dl>

<p>
Besides the additive <strong>proportional, integral</strong> and <strong>derivative</strong>
part of this controller, the following features are present:
</p>
<ul>
<li> The output of this controller is limited. If the controller is
     in its limits, anti-windup compensation is activated to drive
     the integrator state to zero.</li>
<li> The high-frequency gain of the derivative part is limited
     to avoid excessive amplification of measurement noise.</li>
<li> Setpoint weighting is present, which allows to weight
     the setpoint in the proportional and the derivative part
     independently from the measurement. The controller will respond
     to load disturbances and measurement noise independently of this setting
     (parameters wp, wd). However, setpoint changes will depend on this
     setting. For example, it is useful to set the setpoint weight wd
     for the derivative part to zero, if steps may occur in the
     setpoint signal.</li>
<li> Optional feed-forward. It is possible to add a feed-forward signal.
     The feed-forward signal is added before limitation.</li>
</ul>

<p>
The parameters of the controller can be manually adjusted by performing
simulations of the closed loop system (= controller + plant connected
together) and using the following strategy:
</p>

<ol>
<li> Set very large limits, e.g., yMax = Modelica.Constants.inf</li>
<li> Select a <strong>P</strong>-controller and manually enlarge parameter <strong>k</strong>
     (the total gain of the controller) until the closed-loop response
     cannot be improved any more.</li>
<li> Select a <strong>PI</strong>-controller and manually adjust parameters
     <strong>k</strong> and <strong>Ti</strong> (the time constant of the integrator).
     The first value of Ti can be selected, such that it is in the
     order of the time constant of the oscillations occurring with
     the P-controller. If, e.g., vibrations in the order of T=10 ms
     occur in the previous step, start with Ti=0.01 s.</li>
<li> If you want to make the reaction of the control loop faster
     (but probably less robust against disturbances and measurement noise)
     select a <strong>PID</strong>-Controller and manually adjust parameters
     <strong>k</strong>, <strong>Ti</strong>, <strong>Td</strong> (time constant of derivative block).</li>
<li> Set the limits yMax and yMin according to your specification.</li>
<li> Perform simulations such that the output of the PID controller
     goes in its limits. Tune <strong>Ni</strong> (Ni*Ti is the time constant of
     the anti-windup compensation) such that the input to the limiter
     block (= limiter.u) goes quickly enough back to its limits.
     If Ni is decreased, this happens faster. If Ni=infinity, the
     anti-windup compensation is switched off and the controller works bad.</li>
</ol>

<p>
<strong>Initialization</strong>
</p>

<p>
This block can be initialized in different
ways controlled by parameter <strong>initType</strong>. The possible
values of initType are defined in
<a href=\"modelica://Modelica.Blocks.Types.InitPID\">Modelica.Blocks.Types.InitPID</a>.
This type is identical to
<a href=\"modelica://Modelica.Blocks.Types.Init\">Types.Init</a>,
with the only exception that the additional option
<strong>DoNotUse_InitialIntegratorState</strong> is added for
backward compatibility reasons (= integrator is initialized with
InitialState whereas differential part is initialized with
NoInit which was the initialization in version 2.2 of the Modelica
standard library).
</p>

<p>
Based on the setting of initType, the integrator (I) and derivative (D)
blocks inside the PID controller are initialized according to the following table:
</p>

<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>initType</strong></td>
      <td><strong>I.initType</strong></td>
      <td><strong>D.initType</strong></td></tr>

  <tr><td><strong>NoInit</strong></td>
      <td>NoInit</td>
      <td>NoInit</td></tr>

  <tr><td><strong>SteadyState</strong></td>
      <td>SteadyState</td>
      <td>SteadyState</td></tr>

  <tr><td><strong>InitialState</strong></td>
      <td>InitialState</td>
      <td>InitialState</td></tr>

  <tr><td><strong>InitialOutput</strong><br>
          and initial equation: y = y_start</td>
      <td>NoInit</td>
      <td>SteadyState</td></tr>

  <tr><td><strong>DoNotUse_InitialIntegratorState</strong></td>
      <td>InitialState</td>
      <td>NoInit</td></tr>
</table>

<p>
In many cases, the most useful initial condition is
<strong>SteadyState</strong> because initial transients are then no longer
present. If initType = InitPID.SteadyState, then in some
cases difficulties might occur. The reason is the
equation of the integrator:
</p>

<pre>
   <strong>der</strong>(y) = k*u;
</pre>

<p>
The steady state equation \"der(x)=0\" leads to the condition that the input u to the
integrator is zero. If the input u is already (directly or indirectly) defined
by another initial condition, then the initialization problem is <strong>singular</strong>
(has none or infinitely many solutions). This situation occurs often
for mechanical systems, where, e.g., u = desiredSpeed - measuredSpeed and
since speed is both a state and a derivative, it is natural to
initialize it with zero. As sketched this is, however, not possible.
The solution is to not initialize u_m or the variable that is used
to compute u_m by an algebraic equation.
</p>

<p>
When initializing in steady-state, homotopy-based initialization can help the convergence of the solver,
by using a simplified model a the beginning of the solution process. Different options are available.
</p>

<ul>
<li><strong>homotopyType=Linear</strong> (default): the limitations are removed from the simplified model,
making it linear. Use this if you know that the controller will not be saturated at steady state.</li>
<li><strong>homotopyType=UpperLimit</strong>: if it is known a priori the controller will be stuck at the upper
limit yMax, this option assumes y = yMax as a simplified model.</li>
<li><strong>homotopyType=LowerLimit</strong>: if it is known a priori the controller will be stuck at the lower
limit yMin, this option assumes y = yMin as a simplified model.</li>
<li><strong>homotopyType=NoHomotopy</strong>: this option does not apply any simplification and keeps the
limiter active throughout the homotopy transformation. Use this if it is unknown whether the controller
is saturated or not at initialization and if the limitations on the output must be enforced throughout
the entire homotopy transformation.</li>
</ul>

<p>
The parameter <strong>limitAtInit</strong> is obsolete since MSL 3.2.2 and only kept for backwards compatibility.
</p>
</html>"));
      end LimPID;
      annotation (
        Documentation(info="<html>
<p>
This package contains basic <strong>continuous</strong> input/output blocks
described by differential equations.
</p>

<p>
All blocks of this package can be initialized in different
ways controlled by parameter <strong>initType</strong>. The possible
values of initType are defined in
<a href=\"modelica://Modelica.Blocks.Types.Init\">Modelica.Blocks.Types.Init</a>:
</p>

<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>Name</strong></td>
      <td><strong>Description</strong></td></tr>

  <tr><td><strong>Init.NoInit</strong></td>
      <td>no initialization (start values are used as guess values with fixed=false)</td></tr>

  <tr><td><strong>Init.SteadyState</strong></td>
      <td>steady state initialization (derivatives of states are zero)</td></tr>

  <tr><td><strong>Init.InitialState</strong></td>
      <td>Initialization with initial states</td></tr>

  <tr><td><strong>Init.InitialOutput</strong></td>
      <td>Initialization with initial outputs (and steady state of the states if possible)</td></tr>
</table>

<p>
For backward compatibility reasons the default of all blocks is
<strong>Init.NoInit</strong>, with the exception of Integrator and LimIntegrator
where the default is <strong>Init.InitialState</strong> (this was the initialization
defined in version 2.2 of the Modelica standard library).
</p>

<p>
In many cases, the most useful initial condition is
<strong>Init.SteadyState</strong> because initial transients are then no longer
present. The drawback is that in combination with a non-linear
plant, non-linear algebraic equations occur that might be
difficult to solve if appropriate guess values for the
iteration variables are not provided (i.e., start values with fixed=false).
However, it is often already useful to just initialize
the linear blocks from the Continuous blocks library in SteadyState.
This is uncritical, because only linear algebraic equations occur.
If Init.NoInit is set, then the start values for the states are
interpreted as <strong>guess</strong> values and are propagated to the
states with fixed=<strong>false</strong>.
</p>

<p>
Note, initialization with Init.SteadyState is usually difficult
for a block that contains an integrator
(Integrator, LimIntegrator, PI, PID, LimPID).
This is due to the basic equation of an integrator:
</p>

<pre>
  <strong>initial equation</strong>
     <strong>der</strong>(y) = 0;   // Init.SteadyState
  <strong>equation</strong>
     <strong>der</strong>(y) = k*u;
</pre>

<p>
The steady state equation leads to the condition that the input to the
integrator is zero. If the input u is already (directly or indirectly) defined
by another initial condition, then the initialization problem is <strong>singular</strong>
(has none or infinitely many solutions). This situation occurs often
for mechanical systems, where, e.g., u = desiredSpeed - measuredSpeed and
since speed is both a state and a derivative, it is always defined by
Init.InitialState or Init.SteadyState initialization.
</p>

<p>
In such a case, <strong>Init.NoInit</strong> has to be selected for the integrator
and an additional initial equation has to be added to the system
to which the integrator is connected. E.g., useful initial conditions
for a 1-dim. rotational inertia controlled by a PI controller are that
<strong>angle</strong>, <strong>speed</strong>, and <strong>acceleration</strong> of the inertia are zero.
</p>

</html>"),     Icon(graphics={Line(
              origin={0.061,4.184},
              points={{81.939,36.056},{65.362,36.056},{14.39,-26.199},{-29.966,
                  113.485},{-65.374,-61.217},{-78.061,-78.184}},
              color={95,95,95},
              smooth=Smooth.Bezier)}));
    end Continuous;

    package Interfaces "Library of connectors and partial models for input/output blocks"
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

      partial block SVcontrol "Single-Variable continuous controller"
        extends Modelica.Blocks.Icons.Block;

        RealInput u_s "Connector of setpoint input signal" annotation (Placement(
              transformation(extent={{-140,-20},{-100,20}})));
        RealInput u_m "Connector of measurement input signal" annotation (Placement(
              transformation(
              origin={0,-120},
              extent={{20,-20},{-20,20}},
              rotation=270)));
        RealOutput y "Connector of actuator output signal" annotation (Placement(
              transformation(extent={{100,-10},{120,10}})));
        annotation (Diagram(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={Text(
                  extent={{-102,34},{-142,24}},
                  textString="(setpoint)",
                  lineColor={0,0,255}),Text(
                  extent={{100,24},{140,14}},
                  textString="(actuator)",
                  lineColor={0,0,255}),Text(
                  extent={{-83,-112},{-33,-102}},
                  textString=" (measurement)",
                  lineColor={0,0,255})}), Documentation(info="<html>
<p>
Block has two continuous Real input signals and one
continuous Real output signal. The block is designed
to be used as base class for a corresponding controller.
</p>
</html>"));
      end SVcontrol;

      partial block partialBooleanSO "Partial block with 1 output Boolean signal"

        Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        extends Modelica.Blocks.Icons.PartialBooleanBlock;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Ellipse(
                extent={{71,7},{85,-7}},
                lineColor=DynamicSelect({235,235,235}, if y > 0.5 then {0,255,0}
                     else {235,235,235}),
                fillColor=DynamicSelect({235,235,235}, if y > 0.5 then {0,255,0}
                     else {235,235,235}),
                fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
Block has one continuous Boolean output signal
with a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

      end partialBooleanSO;
      annotation (Documentation(info="<html>
<p>
This package contains interface definitions for
<strong>continuous</strong> input/output blocks with Real,
Integer and Boolean signals. Furthermore, it contains
partial models for continuous and discrete blocks.
</p>

</html>",     revisions="<html>
<ul>
<li><em>Oct. 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Added several new interfaces.</li>
<li><em>Oct. 24, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       RealInputSignal renamed to RealInput. RealOutputSignal renamed to
       output RealOutput. GraphBlock renamed to BlockIcon. SISOreal renamed to
       SISO. SOreal renamed to SO. I2SOreal renamed to M2SO.
       SignalGenerator renamed to SignalSource. Introduced the following
       new models: MIMO, MIMOs, SVcontrol, MVcontrol, DiscreteBlockIcon,
       DiscreteBlock, DiscreteSISO, DiscreteMIMO, DiscreteMIMOs,
       BooleanBlockIcon, BooleanSISO, BooleanSignalSource, MI2BooleanMOs.</li>
<li><em>June 30, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
    end Interfaces;

    package Logical "Library of components with Boolean input and output signals"
      extends Modelica.Icons.Package;

      block Switch "Switch between two Real signals"
        extends Modelica.Blocks.Icons.PartialBooleanBlock;
        Blocks.Interfaces.RealInput u1 "Connector of first Real input signal"
          annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
        Blocks.Interfaces.BooleanInput u2 "Connector of Boolean input signal"
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Blocks.Interfaces.RealInput u3 "Connector of second Real input signal"
          annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
        Blocks.Interfaces.RealOutput y "Connector of Real output signal"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      equation
        y = if u2 then u1 else u3;
        annotation (
          defaultComponentName="switch1",
          Documentation(info="<html>
<p>The Logical.Switch switches, depending on the
logical connector u2 (the middle connector)
between the two possible input signals
u1 (upper connector) and u3 (lower connector).</p>
<p>If u2 is <strong>true</strong>, the output signal y is set equal to
u1, else it is set equal to u3.</p>
</html>"),Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{12,0},{100,0}},
                color={0,0,127}),
              Line(points={{-100,0},{-40,0}},
                color={255,0,255}),
              Line(points={{-100,-80},{-40,-80},{-40,-80}},
                color={0,0,127}),
              Line(points={{-40,12},{-40,-12}},
                color={255,0,255}),
              Line(points={{-100,80},{-38,80}},
                color={0,0,127}),
              Line(points=DynamicSelect({{-38,80},{6,2}}, if u2 then {{-38,80},{6,2}} else {{-38,-80},{6,2}}),
                color={0,0,127},
                thickness=1),
              Ellipse(lineColor={0,0,255},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                extent={{2,-8},{18,8}})}));
      end Switch;
      annotation (Documentation(info="<html>
<p>
This package provides blocks with Boolean input and output signals
to describe logical networks. A typical example for a logical
network built with package Logical is shown in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/LogicalNetwork1.png\"
     alt=\"LogicalNetwork1.png\">
</p>

<p>
The actual value of Boolean input and/or output signals is displayed
in the respective block icon as \"circle\", where \"white\" color means
value <strong>false</strong> and \"green\" color means value <strong>true</strong>. These
values are visualized in a diagram animation.
</p>
</html>"),     Icon(graphics={Line(
              points={{-86,-22},{-50,-22},{-50,22},{48,22},{48,-22},{88,-24}},
              color={255,0,255})}));
    end Logical;

    package Math "Library of Real mathematical functions as input/output blocks"
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
This block computes output <em>y</em> as
<em>product</em> of gain <em>k</em> with the
input <em>u</em>:
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

      block Add "Output the sum of the two inputs"
        extends Interfaces.SI2SO;

        parameter Real k1=+1 "Gain of input signal 1";
        parameter Real k2=+1 "Gain of input signal 2";

      equation
        y = k1*u1 + k2*u2;
        annotation (
          Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as <em>sum</em> of the
two input signals <strong>u1</strong> and <strong>u2</strong>:
</p>
<pre>
    <strong>y</strong> = k1*<strong>u1</strong> + k2*<strong>u2</strong>;
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
                0,0,127}),Line(points={{-100,-60},{-74,-28},{-42,-28}}, color={0,0,127}),
                Ellipse(extent={{-50,50},{50,-50}}, lineColor={0,0,127}),Line(
                points={{50,0},{100,0}}, color={0,0,127}),Text(
                  extent={{-36,38},{40,-30}},
                  textString="+"),Text(
                  extent={{-100,52},{5,92}},
                  textString="k1"),Text(
                  extent={{-100,-52},{5,-92}},
                  textString="k2")}));
      end Add;

      block Add3 "Output the sum of the three inputs"
        extends Modelica.Blocks.Icons.Block;

        parameter Real k1=+1 "Gain of input signal 1";
        parameter Real k2=+1 "Gain of input signal 2";
        parameter Real k3=+1 "Gain of input signal 3";
        Interfaces.RealInput u1 "Connector of Real input signal 1" annotation (
            Placement(transformation(extent={{-140,60},{-100,100}})));
        Interfaces.RealInput u2 "Connector of Real input signal 2" annotation (
            Placement(transformation(extent={{-140,-20},{-100,20}})));
        Interfaces.RealInput u3 "Connector of Real input signal 3" annotation (
            Placement(transformation(extent={{-140,-100},{-100,-60}})));
        Interfaces.RealOutput y "Connector of Real output signal" annotation (
            Placement(transformation(extent={{100,-10},{120,10}})));

      equation
        y = k1*u1 + k2*u2 + k3*u3;
        annotation (
          Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as <em>sum</em> of the
three input signals <strong>u1</strong>, <strong>u2</strong> and <strong>u3</strong>:
</p>
<pre>
    <strong>y</strong> = k1*<strong>u1</strong> + k2*<strong>u2</strong> + k3*<strong>u3</strong>;
</pre>
<p>
Example:
</p>
<pre>
     parameter:   k1= +2, k2= -3, k3=1;

  results in the following equations:

     y = 2 * u1 - 3 * u2 + u3;
</pre>

</html>"),Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}), graphics={
              Text(
                extent={{-100,50},{5,90}},
                textString="%k1"),
              Text(
                extent={{-100,-20},{5,20}},
                textString="%k2"),
              Text(
                extent={{-100,-50},{5,-90}},
                textString="%k3"),
              Text(
                extent={{2,36},{100,-44}},
                textString="+")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                  extent={{-100,-100},{100,100}},
                  lineColor={0,0,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Text(
                  extent={{-100,50},{5,90}},
                  textString="k1"),Text(
                  extent={{-100,-20},{5,20}},
                  textString="k2"),Text(
                  extent={{-100,-50},{5,-90}},
                  textString="k3"),Text(
                  extent={{2,46},{100,-34}},
                  textString="+")}));
      end Add3;

      block Product "Output product of the two inputs"
        extends Interfaces.SI2SO;

      equation
        y = u1*u2;
        annotation (
          Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <em>product</em> of the two inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<pre>
    y = u1 * u2;
</pre>

</html>"),Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{-100,60},{-40,60},{-30,40}}, color={0,0,127}),
              Line(points={{-100,-60},{-40,-60},{-30,-40}}, color={0,0,127}),
              Line(points={{50,0},{100,0}}, color={0,0,127}),
              Line(points={{-30,0},{30,0}}),
              Line(points={{-15,25.99},{15,-25.99}}),
              Line(points={{-15,-25.99},{15,25.99}}),
              Ellipse(lineColor={0,0,127}, extent={{-50,-50},{50,50}})}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                  extent={{-100,-100},{100,100}},
                  lineColor={0,0,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(points={{-100,60},{-40,60},{-30,
                40}}, color={0,0,255}),Line(points={{-100,-60},{-40,-60},{-30,-40}},
                color={0,0,255}),Line(points={{50,0},{100,0}}, color={0,0,255}),
                Line(points={{-30,0},{30,0}}),Line(points={{-15,
                25.99},{15,-25.99}}),Line(points={{-15,-25.99},{15,
                25.99}}),Ellipse(extent={{-50,50},{50,-50}},
                lineColor={0,0,255})}));
      end Product;

      block Division "Output first input divided by second input"
        extends Interfaces.SI2SO;

      equation
        y = u1/u2;
        annotation (
          Documentation(info="<html>
<p>
This block computes the output <strong>y</strong>
by <em>dividing</em> the two inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<pre>
    y = u1 / u2;
</pre>

</html>"),Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{50,0},{100,0}}, color={0,0,127}),
              Line(points={{-30,0},{30,0}}),
              Ellipse(fillPattern=FillPattern.Solid, extent={{-5,20},{5,30}}),
              Ellipse(fillPattern=FillPattern.Solid, extent={{-5,-30},{5,-20}}),
              Ellipse(lineColor={0,0,127}, extent={{-50,-50},{50,50}}),
              Line(points={{-100,60},{-66,60},{-40,30}}, color={0,0,127}),
              Line(points={{-100,-60},{0,-60},{0,-50}}, color={0,0,127}),
              Text(
                extent={{-60,94},{90,54}},
                lineColor={128,128,128},
                textString="u1 / u2")}),
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                  extent={{-100,-100},{100,100}},
                  lineColor={0,0,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(points={{50,0},{100,0}},
                color={0,0,255}),Line(points={{-30,0},{30,0}}),
                Ellipse(
                  extent={{-5,20},{5,30}},
                  fillPattern=FillPattern.Solid),Ellipse(
                  extent={{-5,-20},{5,-30}},
                  fillPattern=FillPattern.Solid),Ellipse(extent={{-50,50},{50,-50}},
                lineColor={0,0,255}),Line(points={{-100,60},{-66,60},{-40,30}},
                color={0,0,255}),Line(points={{-100,-60},{0,-60},{0,-50}}, color={0,
                0,255})}));
      end Division;

      block RealToBoolean "Convert Real to Boolean signal"

        Blocks.Interfaces.RealInput u "Connector of Real input signal" annotation (
            Placement(transformation(extent={{-140,-20},{-100,20}})));
        extends Interfaces.partialBooleanSO;
        parameter Real threshold=0.5
          "Output signal y is true, if input u >= threshold";

      equation
        y = u >= threshold;
        annotation (Documentation(info="<html>
<p>
This block computes the Boolean output <strong>y</strong>
from the Real input <strong>u</strong> by the equation:
</p>

<pre>    y = u &ge; threshold;
</pre>

<p>
where <strong>threshold</strong> is a parameter.
</p>
</html>"),     Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={
              Text(
                extent={{-86,92},{-6,10}},
                lineColor={0,0,127},
                textString="R"),
              Polygon(
                points={{-12,-46},{-32,-26},{-32,-36},{-64,-36},{-64,-56},{-32,-56},
                    {-32,-66},{-12,-46}},
                lineColor={255,0,255},
                fillColor={255,0,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{8,-4},{92,-94}},
                lineColor={255,0,255},
                textString="B")}));
      end RealToBoolean;

      block Max "Pass through the largest signal"
        extends Interfaces.SI2SO;
      equation
        y = max(u1, u2);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Text(
                extent={{-90,36},{90,-36}},
                lineColor={160,160,164},
                textString="max()")}), Documentation(info="<html>
<p>
This block computes the output <strong>y</strong> as <em>maximum</em>
of the two Real inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<pre>    y = <strong>max</strong> ( u1 , u2 );
</pre>
</html>"));
      end Max;

      block Min "Pass through the smallest signal"
        extends Interfaces.SI2SO;
      equation
        y = min(u1, u2);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Text(
                extent={{-90,36},{90,-36}},
                lineColor={160,160,164},
                textString="min()")}), Documentation(info="<html>
<p>
This block computes the output <strong>y</strong> as <em>minimum</em> of
the two Real inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<pre>    y = <strong>min</strong> ( u1 , u2 );
</pre>
</html>"));
      end Min;
      annotation (Documentation(info="<html>
<p>
This package contains basic <strong>mathematical operations</strong>,
such as summation and multiplication, and basic <strong>mathematical
functions</strong>, such as <strong>sqrt</strong> and <strong>sin</strong>, as
input/output blocks. All blocks of this library can be either
connected with continuous blocks or with sampled-data blocks.
</p>
</html>",     revisions="<html>
<ul>
<li><em>August 24, 2016</em>
       by Christian Kral: added WrapAngle</li>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       New blocks added: RealToInteger, IntegerToReal, Max, Min, Edge, BooleanChange, IntegerChange.</li>
<li><em>August 7, 1999</em>
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

    package Nonlinear "Library of discontinuous or non-differentiable algebraic control blocks"
      import Modelica.Blocks.Interfaces;
      extends Modelica.Icons.Package;

          block Limiter "Limit the range of a signal"
            parameter Real uMax(start=1) "Upper limits of input signals";
            parameter Real uMin= -uMax "Lower limits of input signals";
            parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
              annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
            parameter Types.LimiterHomotopy homotopyType = Modelica.Blocks.Types.LimiterHomotopy.Linear "Simplified model for homotopy-based initialization"
              annotation (Evaluate=true, Dialog(group="Initialization"));
            parameter Boolean limitsAtInit=true
              "Has no longer an effect and is only kept for backwards compatibility (the implementation uses now the homotopy operator)"
              annotation (Dialog(tab="Dummy"),Evaluate=true, choices(checkBox=true));
            extends Interfaces.SISO;
      protected
            Real simplifiedExpr "Simplified expression for homotopy-based initialization";

          equation
            assert(uMax >= uMin, "Limiter: Limits must be consistent. However, uMax (=" + String(uMax) +
                                 ") < uMin (=" + String(uMin) + ")");
            simplifiedExpr = (if homotopyType == Types.LimiterHomotopy.Linear then u
                              else if homotopyType == Types.LimiterHomotopy.UpperLimit then uMax
                              else if homotopyType == Types.LimiterHomotopy.LowerLimit then uMin
                              else 0);
            if strict then
              if homotopyType == Types.LimiterHomotopy.NoHomotopy then
                y = smooth(0, noEvent(if u > uMax then uMax else if u < uMin then uMin else u));
              else
                y = homotopy(actual = smooth(0, noEvent(if u > uMax then uMax else if u < uMin then uMin else u)),
                             simplified=simplifiedExpr);
              end if;
            else
              if homotopyType == Types.LimiterHomotopy.NoHomotopy then
                y = smooth(0,if u > uMax then uMax else if u < uMin then uMin else u);
              else
                y = homotopy(actual = smooth(0,if u > uMax then uMax else if u < uMin then uMin else u),
                             simplified=simplifiedExpr);
              end if;
            end if;
            annotation (
              Documentation(info="<html>
<p>
The Limiter block passes its input signal as output signal
as long as the input is within the specified upper and lower
limits. If this is not the case, the corresponding limits are passed
as output.
</p>
<p>
The parameter <code>homotopyType</code> in the Advanced tab specifies the
simplified behaviour if homotopy-based initialization is used:
</p>
<ul>
<li><code>NoHomotopy</code>: the actual expression with limits is used</li>
<li><code>Linear</code>: a linear behaviour y = u is assumed (default option)</li>
<li><code>UpperLimit</code>: it is assumed that the output is stuck at the upper limit u = uMax</li>
<li><code>LowerLimit</code>: it is assumed that the output is stuck at the lower limit u = uMin</li>
</ul>
<p>
If it is known a priori in which region the input signal will be located, this option can help
a lot by removing one strong nonlinearity from the initialization problem.
</p>
</html>"),     Icon(coordinateSystem(
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
                textString="uMax=%uMax"),
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
This package contains <strong>discontinuous</strong> and
<strong>non-differentiable, algebraic</strong> input/output blocks.
</p>
</html>",     revisions="<html>
<ul>
<li><em>October 21, 2002</em>
       by Christian Schweiger:<br>
       New block VariableLimiter added.</li>
<li><em>August 22, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.
</li>
</ul>
</html>"),     Icon(graphics={Line(points={{-80,-66},{-26,-66},{28,52},{88,52}},
                color={95,95,95})}));
    end Nonlinear;

    package Sources "Library of signal source blocks generating Real, Integer and Boolean signals"
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
                fillColor={235,235,235},
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised),
              Text(
                extent={{-96,15},{96,-15}},
                textString="%y"),
              Text(
                extent={{-150,90},{150,50}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(info="<html>
<p>
The (time varying) Real output signal of this block can be defined in its
parameter menu via variable <strong>y</strong>. The purpose is to support the
easy definition of Real expressions in a block diagram. For example,
in the y-menu the definition \"if time &lt; 1 then 0 else 1\" can be given in order
to define that the output signal is one, if time &ge; 1 and otherwise
it is zero. Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
variable <strong>y</strong> is both a variable and a connector.
</p>
</html>"));

      end RealExpression;

      block Constant "Generate constant signal of type Real"
        parameter Real k(start=1) "Constant output value"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Constant.png"));
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
                textString="y"),
              Text(
                extent={{70,-80},{94,-100}},
                textString="time"),
              Text(
                extent={{-101,8},{-81,-12}},
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
      annotation (Documentation(info="<html>
<p>
This package contains <strong>source</strong> components, i.e., blocks which
have only output signals. These blocks are used as signal generators
for Real, Integer and Boolean signals.
</p>

<p>
All Real source signals (with the exception of the Constant source)
have at least the following two parameters:
</p>

<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>offset</strong></td>
      <td>Value which is added to the signal</td>
  </tr>
  <tr><td><strong>startTime</strong></td>
      <td>Start time of signal. For time &lt; startTime,
                the output y is set to offset.</td>
  </tr>
</table>

<p>
The <strong>offset</strong> parameter is especially useful in order to shift
the corresponding source, such that at initial time the system
is stationary. To determine the corresponding value of offset,
usually requires a trimming calculation.
</p>
</html>",     revisions="<html>
<ul>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Integer sources added. Step, TimeTable and BooleanStep slightly changed.</li>
<li><em>Nov. 8, 1999</em>
       by <a href=\"mailto:christoph@clauss-it.com\">Christoph Clau&szlig;</a>,
       <a href=\"mailto:Andre.Schneider@eas.iis.fraunhofer.de\">Andre.Schneider@eas.iis.fraunhofer.de</a>,
       <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       New sources: Exponentials, TimeTable. Trapezoid slightly enhanced
       (nperiod=-1 is an infinite number of periods).</li>
<li><em>Oct. 31, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       <a href=\"mailto:christoph@clauss-it.com\">Christoph Clau&szlig;</a>,
       <a href=\"mailto:Andre.Schneider@eas.iis.fraunhofer.de\">Andre.Schneider@eas.iis.fraunhofer.de</a>,
       All sources vectorized. New sources: ExpSine, Trapezoid,
       BooleanConstant, BooleanStep, BooleanPulse, SampleTrigger.
       Improved documentation, especially detailed description of
       signals in diagram layer.</li>
<li><em>June 29, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
    end Sources;

    package Types "Library of constants, external objects and types with choices, especially to build menus"
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

        type InitPID = enumeration(
          NoInit
            "No initialization (start values are used as guess values with fixed=false)",
          SteadyState
            "Steady state initialization (derivatives of states are zero)",
          InitialState "Initialization with initial states",
          InitialOutput
            "Initialization with initial outputs (and steady state of the states if possible)",
          DoNotUse_InitialIntegratorState
            "Do not use, only for backward compatibility (initialize only integrator state)")
        "Enumeration defining initialization of PID and LimPID blocks" annotation (
          Evaluate=true, Documentation(info="<html>
<p>
This initialization type is identical to <a href=\"modelica://Modelica.Blocks.Types.Init\">Types.Init</a> and has just one
additional option <strong><code>DoNotUse_InitialIntegratorState</code></strong>. This option
is introduced in order that the default initialization for the
<code>Continuous.PID</code> and <code>Continuous.LimPID</code> blocks are backward
compatible. In Modelica 2.2, the integrators have been initialized
with their given states where as the D-part has not been initialized.
The option <strong><code>DoNotUse_InitialIntegratorState</code></strong> leads to this
initialization definition.
</p>

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
    <dt><code><strong>DoNotUse_InitialIntegratorState</strong></code></dt>
      <dd>Do not use, only for backward compatibility (initialize only integrator state)</dd>
  </dl>
</html>"));

       type SimpleController = enumeration(
          P "P controller",
          PI "PI controller",
          PD "PD controller",
          PID "PID controller")
        "Enumeration defining P, PI, PD, or PID simple controller type" annotation (
         Evaluate=true);

      type LimiterHomotopy = enumeration(
          NoHomotopy "Homotopy is not used",
          Linear "Simplified model without limits",
          UpperLimit "Simplified model fixed at upper limit",
          LowerLimit "Simplified model fixed at lower limit")
        "Enumeration defining use of homotopy in limiter components" annotation (Evaluate=true);
      annotation (Documentation(info="<html>
<p>
In this package <strong>types</strong>, <strong>constants</strong> and <strong>external objects</strong> are defined that are used
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

      partial block PartialBooleanBlock "Basic graphical layout of logical block"

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Rectangle(
                extent={{-100,100},{100,-100}},
                fillColor={210,210,210},
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised), Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(info="<html>
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
<dt><strong>Main Author:</strong></dt>
<dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a><br>
    Deutsches Zentrum f&uuml;r Luft und Raumfahrt e. V. (DLR)<br>
    Oberpfaffenhofen<br>
    Postfach 1116<br>
    D-82230 Wessling<br>
    email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a><br></dd>
</dl>
<p>
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>",   revisions="<html>
<ul>
<li><em>June 23, 2004</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Introduced new block connectors and adapted all blocks to the new connectors.
       Included subpackages Continuous, Discrete, Logical, Nonlinear from
       package ModelicaAdditions.Blocks.
       Included subpackage ModelicaAdditions.Table in Modelica.Blocks.Sources
       and in the new package Modelica.Blocks.Tables.
       Added new blocks to Blocks.Sources and Blocks.Logical.
       </li>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       New subpackage Examples, additional components.
       </li>
<li><em>June 20, 2000</em>
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
<li><em>Sept. 18, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Renamed to Blocks. New subpackages Math, Nonlinear.
       Additional components in subpackages Interfaces, Continuous
       and Sources.</li>
<li><em>June 30, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
  end Blocks;

  package Fluid "Library of 1-dim. thermo-fluid flow models using the Modelica.Media media description"
    extends Modelica.Icons.Package;
  import SI = Modelica.SIunits;
  import Cv = Modelica.SIunits.Conversions;

    model System
      "System properties and default values (ambient, flow direction, initialization)"

      package Medium = Modelica.Media.Interfaces.PartialMedium
        "Medium model for default start values"
          annotation (choicesAllMatching = true);
      parameter Modelica.SIunits.AbsolutePressure p_ambient=101325
        "Default ambient pressure"
        annotation(Dialog(group="Environment"));
      parameter Modelica.SIunits.Temperature T_ambient=293.15
        "Default ambient temperature"
        annotation(Dialog(group="Environment"));
      parameter Modelica.SIunits.Acceleration g=Modelica.Constants.g_n
        "Constant gravity acceleration"
        annotation(Dialog(group="Environment"));

      // Assumptions
      parameter Boolean allowFlowReversal = true
        "= false to restrict to design flow direction (port_a -> port_b)"
        annotation(Dialog(tab="Assumptions"), Evaluate=true);
      parameter Modelica.Fluid.Types.Dynamics energyDynamics=
        Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
        "Default formulation of energy balances"
        annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
      parameter Modelica.Fluid.Types.Dynamics massDynamics=
        energyDynamics "Default formulation of mass balances"
        annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
      final parameter Modelica.Fluid.Types.Dynamics substanceDynamics=
        massDynamics "Default formulation of substance balances"
        annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
      final parameter Modelica.Fluid.Types.Dynamics traceDynamics=
        massDynamics "Default formulation of trace substance balances"
        annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
      parameter Modelica.Fluid.Types.Dynamics momentumDynamics=
        Modelica.Fluid.Types.Dynamics.SteadyState
        "Default formulation of momentum balances, if options available"
        annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));

      // Initialization
      parameter Modelica.SIunits.MassFlowRate m_flow_start = 0
        "Default start value for mass flow rates"
        annotation(Dialog(tab = "Initialization"));
      parameter Modelica.SIunits.AbsolutePressure p_start = p_ambient
        "Default start value for pressures"
        annotation(Dialog(tab = "Initialization"));
      parameter Modelica.SIunits.Temperature T_start = T_ambient
        "Default start value for temperatures"
        annotation(Dialog(tab = "Initialization"));
      // Advanced
      parameter Boolean use_eps_Re = false
        "= true to determine turbulent region automatically using Reynolds number"
        annotation(Evaluate=true, Dialog(tab = "Advanced"));
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal = if use_eps_Re then 1 else 1e2*m_flow_small
        "Default nominal mass flow rate"
        annotation(Dialog(tab="Advanced", enable = use_eps_Re));
      parameter Real eps_m_flow(min=0) = 1e-4
        "Regularization of zero flow for |m_flow| < eps_m_flow*m_flow_nominal"
        annotation(Dialog(tab = "Advanced", enable = use_eps_Re));
      parameter Modelica.SIunits.AbsolutePressure dp_small(min=0) = 1
        "Default small pressure drop for regularization of laminar and zero flow"
        annotation(Dialog(tab="Advanced", group="Classic", enable = not use_eps_Re));
      parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1e-2
        "Default small mass flow rate for regularization of laminar and zero flow"
        annotation(Dialog(tab = "Advanced", group="Classic", enable = not use_eps_Re));
    initial equation
      //assert(use_eps_Re, "*** Using classic system.m_flow_small and system.dp_small."
      //       + " They do not distinguish between laminar flow and regularization of zero flow."
      //       + " Absolute small values are error prone for models with local nominal values."
      //       + " Moreover dp_small can generally be obtained automatically."
      //       + " Please update the model to new system.use_eps_Re = true  (see system, Advanced tab). ***",
      //       level=AssertionLevel.warning);
      annotation (
        defaultComponentName="system",
        defaultComponentPrefixes="inner",
        missingInnerMessage="
Your model is using an outer \"system\" component but
an inner \"system\" component is not defined.
For simulation drag Modelica.Fluid.System into your model
to specify system properties.",
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,150},{150,110}},
              lineColor={0,0,255},
              textString="%name"),
            Line(points={{-86,-30},{82,-30}}),
            Line(points={{-82,-68},{-52,-30}}),
            Line(points={{-48,-68},{-18,-30}}),
            Line(points={{-14,-68},{16,-30}}),
            Line(points={{22,-68},{52,-30}}),
            Line(points={{74,84},{74,14}}),
            Polygon(
              points={{60,14},{88,14},{74,-18},{60,14}},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{16,20},{60,-18}},
              textString="g"),
            Text(
              extent={{-90,82},{74,50}},
              textString="defaults"),
            Line(
              points={{-82,14},{-42,-20},{2,30}},
              thickness=0.5),
            Ellipse(
              extent={{-10,40},{12,18}},
              pattern=LinePattern.None,
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
 A system component is needed in each fluid model to provide system-wide settings, such as ambient conditions and overall modeling assumptions.
 The system settings are propagated to the fluid models using the inner/outer mechanism.
</p>
<p>
 A model should never directly use system parameters.
 Instead a local parameter should be declared, which uses the global setting as default.
 The only exceptions are:</p>
 <ul>
  <li>the gravity system.g,</li>
  <li>the global system.eps_m_flow, which is used to define a local m_flow_small for the local m_flow_nominal:
      <pre>m_flow_small = system.eps_m_flow*m_flow_nominal</pre>
  </li>
 </ul>
<p>
 The global system.m_flow_small and system.dp_small are classic parameters.
 They do not distinguish between laminar flow and regularization of zero flow.
 Absolute small values are error prone for models with local nominal values.
 Moreover dp_small can generally be obtained automatically.
 Consider using the new system.use_eps_Re = true (see Advanced tab).
</p>
</html>"));
    end System;

    package Vessels "Devices for storing fluid"
        extends Modelica.Icons.VariantsPackage;

        model ClosedVolume
        "Volume of fixed size, closed to the ambient, with inlet/outlet ports"
        import Modelica.Constants.pi;

          // Mass and energy balance, ports
          extends Modelica.Fluid.Vessels.BaseClasses.PartialLumpedVessel(
            final fluidVolume = V,
            vesselArea = pi*(3/4*V)^(2/3),
            heatTransfer(surfaceAreas={4*pi*(3/4*V/pi)^(2/3)}));

          parameter SI.Volume V "Volume";

        equation
          Wb_flow = 0;
          for i in 1:nPorts loop
            vessel_ps_static[i] = medium.p;
          end for;

          annotation (defaultComponentName="volume",
            Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{
                  100,100}}), graphics={Ellipse(
                extent={{-100,100},{100,-100}},
                fillPattern=FillPattern.Sphere,
                fillColor={170,213,255}), Text(
                extent={{-150,12},{150,-18}},
                textString="V=%V")}),
          Documentation(info="<html>
<p>
Ideally mixed volume of constant size with two fluid ports and one medium model.
The flow properties are computed from the upstream quantities, pressures are equal in both nodes and the medium model if <code>use_portsData=false</code>.
Heat transfer through a thermal port is possible, it equals zero if the port remains unconnected.
A spherical shape is assumed for the heat transfer area, with V=4/3*pi*r^3, A=4*pi*r^2.
Ideal heat transfer is assumed per default; the thermal port temperature is equal to the medium temperature.
</p>
<p>
If <code>use_portsData=true</code>, the port pressures represent the pressures just after the outlet (or just before the inlet) in the attached pipe.
The hydraulic resistances <code>portsData.zeta_in</code> and <code>portsData.zeta_out</code> determine the dissipative pressure drop between volume and port depending on
the direction of mass flow. See <a href=\"modelica://Modelica.Fluid.Vessels.BaseClasses.VesselPortsData\">VesselPortsData</a> and <em>[Idelchik, Handbook of Hydraulic Resistance, 2004]</em>.
</p>
</html>"));
        end ClosedVolume;

      package BaseClasses "Base classes used in the Vessels package (only of interest to build new component models)"
        extends Modelica.Icons.BasesPackage;

          partial model PartialLumpedVessel
          "Lumped volume with a vector of fluid ports and replaceable heat transfer model"
            extends Modelica.Fluid.Interfaces.PartialLumpedVolume;

            // Port definitions
            parameter Integer nPorts=0 "Number of ports"
              annotation(Evaluate=true, Dialog(connectorSizing=true, tab="General",group="Ports"));
            VesselFluidPorts_b ports[nPorts](redeclare each package Medium = Medium)
            "Fluid inlets and outlets"
              annotation (Placement(transformation(extent={{-40,-10},{40,10}},
                origin={0,-100})));

            // Port properties
            parameter Boolean use_portsData=true
            "= false to neglect pressure loss and kinetic energy"
              annotation(Evaluate=true, Dialog(tab="General",group="Ports"));
            parameter Modelica.Fluid.Vessels.BaseClasses.VesselPortsData[if use_portsData then nPorts else 0]
            portsData "Data of inlet/outlet ports"
              annotation(Dialog(tab="General",group="Ports",enable= use_portsData));

            parameter Medium.MassFlowRate m_flow_nominal = if system.use_eps_Re then system.m_flow_nominal else 1e2*system.m_flow_small
            "Nominal value for mass flow rates in ports"
              annotation(Dialog(tab="Advanced", group="Port properties"));
            parameter SI.MassFlowRate m_flow_small(min=0) = if system.use_eps_Re then system.eps_m_flow*m_flow_nominal else system.m_flow_small
            "Regularization range at zero mass flow rate"
              annotation(Dialog(tab="Advanced", group="Port properties"));
            parameter Boolean use_Re = system.use_eps_Re
            "= true, if turbulent region is defined by Re, otherwise by m_flow_small"
              annotation(Dialog(tab="Advanced", group="Port properties"), Evaluate=true);

            Medium.EnthalpyFlowRate ports_H_flow[nPorts];
            Medium.MassFlowRate ports_mXi_flow[nPorts,Medium.nXi];
            Medium.MassFlowRate[Medium.nXi] sum_ports_mXi_flow
            "Substance mass flows through ports";
            Medium.ExtraPropertyFlowRate ports_mC_flow[nPorts,Medium.nC];
            Medium.ExtraPropertyFlowRate[Medium.nC] sum_ports_mC_flow
            "Trace substance mass flows through ports";

            // Heat transfer through boundary
            parameter Boolean use_HeatTransfer = false
            "= true to use the HeatTransfer model"
                annotation (Dialog(tab="Assumptions", group="Heat transfer"));
            replaceable model HeatTransfer =
                Modelica.Fluid.Vessels.BaseClasses.HeatTransfer.IdealHeatTransfer
              constrainedby Modelica.Fluid.Vessels.BaseClasses.HeatTransfer.PartialVesselHeatTransfer
            "Wall heat transfer"
                annotation (Dialog(tab="Assumptions", group="Heat transfer",enable=use_HeatTransfer),choicesAllMatching=true);
            HeatTransfer heatTransfer(
              redeclare final package Medium = Medium,
              final n=1,
              final states = {medium.state},
              final use_k = use_HeatTransfer)
                annotation (Placement(transformation(
                  extent={{-10,-10},{30,30}},
                  rotation=90,
                  origin={-50,-10})));
            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort if use_HeatTransfer
              annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

            // Conservation of kinetic energy
            Medium.Density[nPorts] portInDensities
            "densities of the fluid at the device boundary";
            SI.Velocity[nPorts] portVelocities
            "velocities of fluid flow at device boundary";
            SI.EnergyFlowRate[nPorts] ports_E_flow
            "flow of kinetic and potential energy at device boundary";

            // Note: should use fluidLevel_start - portsData.height
            Real[nPorts] s(each start = fluidLevel_max)
            "curve parameters for port flows vs. port pressures; for further details see, Modelica Tutorial: Ideal switching devices";
            Real[nPorts] ports_penetration
            "penetration of port with fluid, depending on fluid level and port diameter";

            // treatment of pressure losses at ports
            SI.Area[nPorts] portAreas = {Modelica.Constants.pi/4*portsData_diameter[i]^2 for i in 1:nPorts};
            Medium.AbsolutePressure[nPorts] vessel_ps_static
            "static pressures inside the vessel at the height of the corresponding ports, zero flow velocity";

            // determination of turbulent region
            constant SI.ReynoldsNumber Re_turbulent = 100 "cf. suddenExpansion";
            SI.MassFlowRate[nPorts] m_flow_turbulent;

        protected
            input SI.Height fluidLevel = 0
            "level of fluid in the vessel for treating heights of ports";
            parameter SI.Height fluidLevel_max = 1
            "maximum level of fluid in the vessel";
            parameter SI.Area vesselArea = Modelica.Constants.inf
            "Area of the vessel used to relate to cross flow area of ports";

            // Treatment of use_portsData=false to neglect portsData and to not require its specification either in this case.
            // Remove portsData conditionally if use_portsData=false. Simplify their use in model equations by always
            // providing portsData_diameter and portsData_height, independent of the use_portsData setting.
            // Note: this moreover serves as work-around if a tool does not support a zero sized portsData record.
            Modelica.Blocks.Interfaces.RealInput[nPorts]
            portsData_diameter_internal = portsData.diameter if use_portsData and nPorts > 0;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_height_internal = portsData.height if use_portsData and nPorts > 0;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_zeta_in_internal = portsData.zeta_in if use_portsData and nPorts > 0;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_zeta_out_internal = portsData.zeta_out if use_portsData and nPorts > 0;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_diameter;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_height;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_zeta_in;
            Modelica.Blocks.Interfaces.RealInput[nPorts] portsData_zeta_out;
            Modelica.Blocks.Interfaces.BooleanInput[nPorts] regularFlow(each start=true);
            Modelica.Blocks.Interfaces.BooleanInput[nPorts] inFlow(each start=false);

          equation
            mb_flow = sum(ports.m_flow);
            mbXi_flow = sum_ports_mXi_flow;
            mbC_flow  = sum_ports_mC_flow;
            Hb_flow = sum(ports_H_flow) + sum(ports_E_flow);
            Qb_flow = heatTransfer.Q_flows[1];

            // Only one connection allowed to a port to avoid unwanted ideal mixing
            for i in 1:nPorts loop
              assert(cardinality(ports[i]) <= 1,"
each ports[i] of volume can at most be connected to one component.
If two or more connections are present, ideal mixing takes
place with these connections, which is usually not the intention
of the modeller. Increase nPorts to add an additional port.
");         end for;
            // Check for correct solution
            assert(fluidLevel <= fluidLevel_max, "Vessel is overflowing (fluidLevel > fluidLevel_max = " + String(fluidLevel) + ")");
            assert(fluidLevel > -1e-6*fluidLevel_max, "Fluid level (= " + String(fluidLevel) + ") is below zero meaning that the solution failed.");

            // Boundary conditions

            // treatment of conditional portsData
            connect(portsData_diameter, portsData_diameter_internal);
            connect(portsData_height, portsData_height_internal);
            connect(portsData_zeta_in, portsData_zeta_in_internal);
            connect(portsData_zeta_out, portsData_zeta_out_internal);
            if not use_portsData then
              portsData_diameter = zeros(nPorts);
              portsData_height = zeros(nPorts);
              portsData_zeta_in = zeros(nPorts);
              portsData_zeta_out = zeros(nPorts);
            end if;

            // actual definition of port variables
            for i in 1:nPorts loop
              portInDensities[i] = Medium.density(Medium.setState_phX(vessel_ps_static[i], inStream(ports[i].h_outflow), inStream(ports[i].Xi_outflow)));
              if use_portsData then
                // dp = 0.5*zeta*d*v*|v|
                // Note: assume vessel_ps_static for portVelocities to avoid algebraic loops for ports.p
                portVelocities[i] = smooth(0, ports[i].m_flow/portAreas[i]/Medium.density(Medium.setState_phX(vessel_ps_static[i], actualStream(ports[i].h_outflow), actualStream(ports[i].Xi_outflow))));
                // Note: the penetration should not go too close to zero as this would prevent a vessel from running empty
                ports_penetration[i] = Utilities.regStep(fluidLevel - portsData_height[i] - 0.1*portsData_diameter[i], 1, 1e-3, 0.1*portsData_diameter[i]);
                m_flow_turbulent[i]=if not use_Re then m_flow_small else
                  max(m_flow_small, (Modelica.Constants.pi/8)*portsData_diameter[i]
                                     *(Medium.dynamicViscosity(Medium.setState_phX(vessel_ps_static[i], inStream(ports[i].h_outflow), inStream(ports[i].Xi_outflow)))
                                       + Medium.dynamicViscosity(medium.state))*Re_turbulent);
              else
                // an infinite port diameter is assumed
                portVelocities[i] = 0;
                ports_penetration[i] = 1;
                m_flow_turbulent[i] = Modelica.Constants.inf;
              end if;

              // fluid flow through ports
              regularFlow[i] = fluidLevel >= portsData_height[i];
              inFlow[i]      = not regularFlow[i] and (s[i] > 0 or portsData_height[i] >= fluidLevel_max);
              if regularFlow[i] then
                // regular operation: fluidLevel is above ports[i]
                // Note: >= covers default values of zero as well
                if use_portsData then
                  /* Without regularization
                 ports[i].p = vessel_ps_static[i] + 0.5*ports[i].m_flow^2/portAreas[i]^2
                              * noEvent(if ports[i].m_flow>0 then zeta_in[i]/portInDensities[i] else -zeta_out[i]/medium.d);
              */

                  ports[i].p = homotopy(vessel_ps_static[i] + (0.5/portAreas[i]^2*Utilities.regSquare2(ports[i].m_flow, m_flow_turbulent[i],
                                               (portsData_zeta_in[i] - 1 + portAreas[i]^2/vesselArea^2)/portInDensities[i]*ports_penetration[i],
                                               (portsData_zeta_out[i] + 1 - portAreas[i]^2/vesselArea^2)/medium.d/ports_penetration[i])),
                                        vessel_ps_static[i]);
                  /*
                // alternative formulation m_flow=f(dp); not allowing the ideal portsData_zeta_in[i]=1 though
                ports[i].m_flow = smooth(2, portAreas[i]*Utilities.regRoot2(ports[i].p - vessel_ps_static[i], dp_small,
                                       2*portInDensities[i]/portsData_zeta_in[i],
                                       2*medium.d/portsData_zeta_out[i]));
              */
                else
                  ports[i].p = vessel_ps_static[i];
                end if;
                s[i] = fluidLevel - portsData_height[i];

              elseif inFlow[i] then
                // ports[i] is above fluidLevel and has inflow
                ports[i].p = vessel_ps_static[i];
                s[i] = ports[i].m_flow;

              else
                // ports[i] is above fluidLevel, preventing outflow
                ports[i].m_flow = 0;
                s[i] = (ports[i].p - vessel_ps_static[i])/Medium.p_default*(portsData_height[i] - fluidLevel);
              end if;

              ports[i].h_outflow  = medium.h;
              ports[i].Xi_outflow = medium.Xi;
              ports[i].C_outflow  = C;

              ports_H_flow[i] = ports[i].m_flow * actualStream(ports[i].h_outflow)
              "Enthalpy flow";
              ports_E_flow[i] = ports[i].m_flow*(0.5*portVelocities[i]*portVelocities[i] + system.g*portsData_height[i])
              "Flow of kinetic and potential energy";
              ports_mXi_flow[i,:] = ports[i].m_flow * actualStream(ports[i].Xi_outflow)
              "Component mass flow";
              ports_mC_flow[i,:]  = ports[i].m_flow * actualStream(ports[i].C_outflow)
              "Trace substance mass flow";
            end for;

            for i in 1:Medium.nXi loop
              sum_ports_mXi_flow[i] = sum(ports_mXi_flow[:,i]);
            end for;

            for i in 1:Medium.nC loop
              sum_ports_mC_flow[i]  = sum(ports_mC_flow[:,i]);
            end for;

            connect(heatPort, heatTransfer.heatPorts[1]) annotation (Line(
                points={{-100,0},{-87,0},{-87,0},{-74,0}}, color={191,0,0}));
           annotation (
            Documentation(info="<html>
<p>
This base class extends PartialLumpedVolume with a vector of fluid ports and a replaceable wall HeatTransfer model.
</p>
<p>
The following modeling assumption are made:</p>
<ul>
<li>homogeneous medium, i.e., phase separation is not taken into account,</li>
<li>no kinetic energy in the fluid, i.e., kinetic energy dissipates into the internal energy,</li>
<li>pressure loss definitions at vessel ports assume incompressible fluid,</li>
<li>outflow of ambient media is prevented at each port assuming check valve behavior.
    If <code>fluidlevel &lt; portsData_height[i]</code> and <code>ports[i].p &lt; vessel_ps_static[i]</code> mass flow at the port is set to 0.</li>
</ul>
<p>
Each port has a (hydraulic) diameter and a height above the bottom of the vessel, which can be configured using the <strong><code>portsData</code></strong> record.
Alternatively the impact of port geometries can be neglected with <code>use_portsData=false</code>. This might be useful for early
design studies. Note that this means to assume an infinite port diameter at the bottom of the vessel.
Pressure drops and heights of the ports as well as kinetic and potential energy fluid entering or leaving the vessel are neglected then.
</p>
<p>
The following variables need to be defined by an extending model:
</p>
<ul>
<li><code>input fluidVolume</code>, the volume of the fluid in the vessel,</li>
<li><code>vessel_ps_static[nPorts]</code>, the static pressures inside the vessel at the height of the corresponding ports, at zero flow velocity, and</li>
<li><code>Wb_flow</code>, work term of the energy balance, e.g., p*der(V) if the volume is not constant or stirrer power.</li>
</ul>
<p>
An extending model should define:
</p>
<ul>
<li><code>parameter vesselArea</code> (default: Modelica.Constants.inf m2), the area of the vessel, to be related to cross flow areas of the ports for the consideration of dynamic pressure effects.</li>
</ul>
<p>
Optionally the fluid level may vary in the vessel, which effects the flow through the ports at configurable <code>portsData_height[nPorts]</code>.
This is why an extending model with varying fluid level needs to define:
</p>
<ul>
<li><code>input fluidLevel (default: 0m)</code>, the level the fluid in the vessel, and</li>
<li><code>parameter fluidLevel_max (default: 1m)</code>, the maximum level that must not be exceeded. Ports at or above fluidLevel_max can only receive inflow.</li>
</ul>
<p>
An extending model should not access the <code>portsData</code> record defined in the configuration dialog,
as an access to <code>portsData</code> may fail for <code>use_portsData=false</code> or <code>nPorts=0</code>.
</p>
<p>
Instead the predefined variables
</p>
<ul>
<li><code>portsData_diameter[nPorts]</code>,</li>
<li><code>portsData_height[nPorts]</code>,</li>
<li><code>portsData_zeta_in[nPorts]</code>, and</li>
<li><code>portsData_zeta_out[nPorts]</code></li>
</ul>
<p>
should be used if these values are needed.
</p>
</html>",           revisions="<html>
<ul>
<li><em>Jan. 2009</em> by R&uuml;diger Franke: extended with
   <ul><li>portsData record and threat configurable port heights,</li>
       <li>consideration of kinetic and potential energy of fluid entering or leaving in energy balance</li>
   </ul>
</li>
<li><em>Dec. 2008</em> by R&uuml;diger Franke: derived from OpenTank, in order to make general use of configurable port diameters</li>
</ul>
</html>"),    Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},
                    {100,100}}), graphics={Text(
                  extent={{-150,110},{150,150}},
                  textString="%name",
                  lineColor={0,0,255})}));
          end PartialLumpedVessel;

      package HeatTransfer "HeatTransfer models for vessels"
        extends Modelica.Icons.Package;

        partial model PartialVesselHeatTransfer
            "Base class for vessel heat transfer models"
          extends Modelica.Fluid.Interfaces.PartialHeatTransfer;

          annotation(Documentation(info="<html>
Base class for vessel heat transfer models.
</html>"),    Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},
                      {100,100}}), graphics={Ellipse(
                    extent={{-60,64},{60,-56}},
                    fillPattern=FillPattern.Sphere,
                    fillColor={232,0,0}), Text(
                    extent={{-38,26},{40,-14}},
                    textString="%name")}));
        end PartialVesselHeatTransfer;

        model IdealHeatTransfer
            "IdealHeatTransfer: Ideal heat transfer without thermal resistance"
          extends PartialVesselHeatTransfer;

        equation
          Ts = heatPorts.T;

          annotation(Documentation(info="<html>
Ideal heat transfer without thermal resistance.
</html>"));
        end IdealHeatTransfer;
        annotation (Documentation(info="<html>
Heat transfer correlations for pipe models
</html>"));
      end HeatTransfer;

        record VesselPortsData "Data to describe inlet/outlet ports at vessels:
    diameter -- Inner (hydraulic) diameter of inlet/outlet port
    height -- Height over the bottom of the vessel
    zeta_out -- Hydraulic resistance out of vessel, default 0.5 for small diameter mounted flush with the wall
    zeta_in -- Hydraulic resistance into vessel, default 1.04 for small diameter mounted flush with the wall"
              extends Modelica.Icons.Record;
          parameter SI.Diameter diameter
            "Inner (hydraulic) diameter of inlet/outlet port";
          parameter SI.Height height = 0 "Height over the bottom of the vessel";
          parameter Real zeta_out(min=0)=0.5
            "Hydraulic resistance out of vessel, default 0.5 for small diameter mounted flush with the wall";
          parameter Real zeta_in(min=0)=1.04
            "Hydraulic resistance into vessel, default 1.04 for small diameter mounted flush with the wall";
          annotation (preferredView="info", Documentation(info="<html>
<h4>Vessel Port Data</h4>
<p>
This record describes the <strong>ports</strong> of a <strong>vessel</strong>. The variables in it are mostly self-explanatory (see list below); only the &zeta;
loss factors are discussed further. All data is quoted from Idelchik (1994).
</p>

<h4>Outlet Coefficients</h4>

<p>
If a <strong>straight pipe with constant cross section is mounted flush with the wall</strong>, its outlet pressure loss coefficient will be <code>&zeta; = 0.5</code> (Idelchik, p. 160, Diagram 3-1, paragraph 2).
</p>
<p>
If a <strong>straight pipe with constant cross section is mounted into a vessel such that the entrance into it is at a distance</strong> <code>b</code> from the wall (inside) the following table can be used. Herein, &delta; is the tube wall thickness (Idelchik, p. 160, Diagram 3-1, paragraph 1).
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Pressure loss coefficients for outlets, entrance at a distance from wall</caption>
  <tr>
    <td></td> <td>   </td><th colspan=\"5\" align=\"center\"> b / D_hyd  </th>
  </tr>
  <tr>
    <td></td> <td>   </td><th> 0.000 </th><th> 0.005 </th><th> 0.020 </th><th> 0.100 </th><th> 0.500-&#8734; </th>
  </tr>
  <tr>
     <th rowspan=\"5\" valign=\"middle\">&delta; / D_hyd</th> <th> 0.000 </th><td> 0.50 </td><td> 0.63  </td><td> 0.73  </td><td> 0.86  </td><td>      1.00     </td>
  </tr>
  <tr>
              <th> 0.008 </th><td> 0.50 </td><td> 0.55  </td><td> 0.62  </td><td> 0.74  </td><td>      0.88     </td>
  </tr>
  <tr>
              <th> 0.016 </th><td> 0.50 </td><td> 0.51  </td><td> 0.55  </td><td> 0.64  </td><td>      0.77     </td>
  </tr>
  <tr>
              <th> 0.024 </th><td> 0.50 </td><td> 0.50  </td><td> 0.52  </td><td> 0.58  </td><td>      0.68     </td>
  </tr>
  <tr>
              <th> 0.040 </th><td> 0.50 </td><td> 0.50  </td><td> 0.51  </td><td> 0.51  </td><td>      0.54     </td>
  </tr>
</table>

<p>
If a <strong>straight pipe with a circular bellmouth inlet (collector) without baffle is mounted flush with the wall</strong> then its pressure loss coefficient can be established from the following table. Herein, r is the radius of the bellmouth inlet surface (Idelchik, p. 164 f., Diagram 3-4, paragraph b)
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Pressure loss coefficients for outlets, bellmouth flush with wall</caption>
  <tr>
    <td></td> <th colspan=\"6\" align=\"center\"> r / D_hyd  </th>
  </tr>
  <tr>
    <td></td> <th> 0.01 </th><th> 0.03 </th><th> 0.05 </th><th> 0.08 </th><th> 0.16 </th><th>&ge;0.20</th>
  </tr>
  <tr>
     <th>&zeta;</th> <td> 0.44 </td><td> 0.31 </td><td> 0.22  </td><td> 0.15  </td><td> 0.06  </td><td>      0.03     </td>
  </tr>
</table>

<p>
If a <strong>straight pipe with a circular bellmouth inlet (collector) without baffle is mounted at a distance from a wall</strong> then its pressure loss coefficient can be established from the following table. Herein, r is the radius of the bellmouth inlet surface (Idelchik, p. 164 f., Diagram 3-4, paragraph a)
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Pressure loss coefficients for outlets, bellmouth at a distance of wall</caption>
  <tr>
    <td></td> <th colspan=\"6\" align=\"center\"> r / D_hyd  </th>
  </tr>
  <tr>
    <td></td> <th> 0.01 </th><th> 0.03 </th><th> 0.05 </th><th> 0.08 </th><th> 0.16 </th><th>&ge;0.20</th>
  </tr>
  <tr>
     <th>&zeta;</th> <td> 0.87 </td><td> 0.61 </td><td> 0.40  </td><td> 0.20  </td><td> 0.06  </td><td>      0.03     </td>
  </tr>
</table>

<h4>Inlet Coefficients</h4>

<p>
If a <strong>straight pipe with constant circular cross section is mounted flush with the wall</strong>, its vessel inlet pressure loss coefficient will be according to the following table (Idelchik, p. 209 f., Diagram 4-2 with <code>A_port/A_vessel = 0</code> and Idelchik, p. 640, Diagram 11-1, graph a). According to the text, <code>m = 9</code> is appropriate for fully developed turbulent flow.
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Pressure loss coefficients for inlets, circular tube flush with wall</caption>
  <tr>
    <td></td> <th colspan=\"6\" align=\"center\"> m  </th>
  </tr>
  <tr>
    <td></td> <th> 1.0 </th><th> 2.0 </th><th> 3.0 </th><th> 4.0 </th><th> 7.0 </th><th>9.0</th>
  </tr>
  <tr>
     <th>&zeta;</th> <td> 2.70 </td><td> 1.50 </td><td> 1.25  </td><td> 1.15  </td><td> 1.06  </td><td>      1.04     </td>
  </tr>
</table>

<p>
For larger port diameters, relative to the area of the vessel, the inlet pressure loss coefficient will be according to the following table (Idelchik, p. 209 f., Diagram 4-2 with <code>m = 7</code>).
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Pressure loss coefficients for inlets, circular tube flush with wall</caption>
  <tr>
    <td></td> <th colspan=\"6\" align=\"center\"> A_port / A_vessel  </th>
  </tr>
  <tr>
    <td></td> <th> 0.0 </th><th> 0.1 </th><th> 0.2 </th><th> 0.4 </th><th> 0.6 </th><th>0.8</th>
  </tr>
  <tr>
     <th>&zeta;</th> <td> 1.04 </td><td> 0.84 </td><td> 0.67  </td><td> 0.39  </td><td> 0.18  </td><td>      0.06     </td>
  </tr>
</table>

<h4>References</h4>

<dl><dt>Idelchik I.E. (1994):</dt>
    <dd><a href=\"http://www.bookfinder.com/dir/i/Handbook_of_Hydraulic_Resistance/0849399084/\"><strong>Handbook
        of Hydraulic Resistance</strong></a>. 3rd edition, Begell House, ISBN
        0-8493-9908-4</dd>
</dl>
</html>"));
        end VesselPortsData;

        connector VesselFluidPorts_b
          "Fluid connector with outlined, large icon to be used for horizontally aligned vectors of FluidPorts (vector dimensions must be added after dragging)"
          extends Interfaces.FluidPort;
          annotation (defaultComponentName="ports_b",
                      Diagram(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-50,-200},{50,200}},
                initialScale=0.2), graphics={
                Text(extent={{-75,130},{75,100}}, textString="%name"),
                Rectangle(
                  extent={{-25,100},{25,-100}}),
                Ellipse(
                  extent={{-22,100},{-10,-100}},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-20,-69},{-12,69}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-6,100},{6,-100}},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{10,100},{22,-100}},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-69},{4,69}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-69},{20,69}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}),
               Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-50,-200},{50,200}},
                initialScale=0.2), graphics={
                Rectangle(
                  extent={{-50,200},{50,-200}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-44,200},{-20,-200}},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-12,200},{12,-200}},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{20,200},{44,-200}},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-39,-118.5},{-25,113}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-7,-118.5},{7,113}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{25,-117.5},{39,114}},
                  lineColor={0,127,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}));
        end VesselFluidPorts_b;
      end BaseClasses;
      annotation (Documentation(info="<html>

</html>"));
    end Vessels;

    package Pipes "Devices for conveying fluid"
        extends Modelica.Icons.VariantsPackage;

      package BaseClasses "Base classes used in the Pipes package (only of interest to build new component models)"
        extends Modelica.Icons.BasesPackage;

        package WallFriction "Different variants for pressure drops due to pipe wall friction"
          extends Modelica.Icons.Package;

          partial package PartialWallFriction
            "Partial wall friction characteristic (base package of all wall friction characteristics)"
            extends Modelica.Icons.Package;
            import Modelica.Constants.pi;

          // Constants to be set in subpackages
            constant Boolean use_mu = true
              "= true, if mu_a/mu_b are used in function, otherwise value is not used";
            constant Boolean use_roughness = true
              "= true, if roughness is used in function, otherwise value is not used";
            constant Boolean use_dp_small = true
              "= true, if dp_small is used in function, otherwise value is not used";
            constant Boolean use_m_flow_small = true
              "= true, if m_flow_small is used in function, otherwise value is not used";
            constant Boolean dp_is_zero = false
              "= true, if no wall friction is present, i.e., dp = 0 (function massFlowRate_dp() cannot be used)";
            constant Boolean use_Re_turbulent = true
              "= true, if Re_turbulent input is used in function, otherwise value is not used";

          // pressure loss characteristic functions
            replaceable partial function massFlowRate_dp
              "Return mass flow rate m_flow as function of pressure loss dp, i.e., m_flow = f(dp), due to wall friction"
              extends Modelica.Icons.Function;

              input SI.Pressure dp "Pressure loss (dp = port_a.p - port_b.p)";
              input SI.Density rho_a "Density at port_a";
              input SI.Density rho_b "Density at port_b";
              input SI.DynamicViscosity mu_a
                "Dynamic viscosity at port_a (dummy if use_mu = false)";
              input SI.DynamicViscosity mu_b
                "Dynamic viscosity at port_b (dummy if use_mu = false)";
              input SI.Length length "Length of pipe";
              input SI.Diameter diameter "Inner (hydraulic) diameter of pipe";
              input SI.Area crossArea = pi*diameter^2/4 "Inner cross section area";
              input SI.Length roughness(min=0) = 2.5e-5
                "Absolute roughness of pipe, with a default for a smooth steel pipe (dummy if use_roughness = false)";
              input SI.AbsolutePressure dp_small = 1
                "Regularization of zero flow if |dp| < dp_small (dummy if use_dp_small = false)";
              input SI.ReynoldsNumber Re_turbulent = 4000
                "Turbulent flow if Re >= Re_turbulent (dummy if use_Re_turbulent = false)";

              output SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
            annotation (Documentation(info="<html>

</html>"));
            end massFlowRate_dp;

            replaceable partial function massFlowRate_dp_staticHead
              "Return mass flow rate m_flow as function of pressure loss dp, i.e., m_flow = f(dp), due to wall friction and static head"
              extends Modelica.Icons.Function;

              input SI.Pressure dp "Pressure loss (dp = port_a.p - port_b.p)";
              input SI.Density rho_a "Density at port_a";
              input SI.Density rho_b "Density at port_b";
              input SI.DynamicViscosity mu_a
                "Dynamic viscosity at port_a (dummy if use_mu = false)";
              input SI.DynamicViscosity mu_b
                "Dynamic viscosity at port_b (dummy if use_mu = false)";
              input SI.Length length "Length of pipe";
              input SI.Diameter diameter "Inner (hydraulic) diameter of pipe";
              input Real g_times_height_ab
                "Gravity times (Height(port_b) - Height(port_a))";
              input SI.Area crossArea = pi*diameter^2/4 "Inner cross section area";
              input SI.Length roughness(min=0) = 2.5e-5
                "Absolute roughness of pipe, with a default for a smooth steel pipe (dummy if use_roughness = false)";
              input SI.AbsolutePressure dp_small=1
                "Regularization of zero flow if |dp| < dp_small (dummy if use_dp_small = false)";
              input SI.ReynoldsNumber Re_turbulent = 4000
                "Turbulent flow if Re >= Re_turbulent (dummy if use_Re_turbulent = false)";

              output SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
              annotation (Documentation(info="<html>

</html>"));
            end massFlowRate_dp_staticHead;

            replaceable partial function pressureLoss_m_flow
              "Return pressure loss dp as function of mass flow rate m_flow, i.e., dp = f(m_flow), due to wall friction"
              extends Modelica.Icons.Function;

              input SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
              input SI.Density rho_a "Density at port_a";
              input SI.Density rho_b "Density at port_b";
              input SI.DynamicViscosity mu_a
                "Dynamic viscosity at port_a (dummy if use_mu = false)";
              input SI.DynamicViscosity mu_b
                "Dynamic viscosity at port_b (dummy if use_mu = false)";
              input SI.Length length "Length of pipe";
              input SI.Diameter diameter "Inner (hydraulic) diameter of pipe";
              input SI.Area crossArea = pi*diameter^2/4 "Inner cross section area";
              input SI.Length roughness(min=0) = 2.5e-5
                "Absolute roughness of pipe, with a default for a smooth steel pipe (dummy if use_roughness = false)";
              input SI.MassFlowRate m_flow_small = 0.01
                "Regularization of zero flow if |m_flow| < m_flow_small (dummy if use_m_flow_small = false)";
              input SI.ReynoldsNumber Re_turbulent = 4000
                "Turbulent flow if Re >= Re_turbulent (dummy if use_Re_turbulent = false)";

              output SI.Pressure dp "Pressure loss (dp = port_a.p - port_b.p)";

            annotation (Documentation(info="<html>

</html>"));
            end pressureLoss_m_flow;

            replaceable partial function pressureLoss_m_flow_staticHead
              "Return pressure loss dp as function of mass flow rate m_flow, i.e., dp = f(m_flow), due to wall friction and static head"
                      extends Modelica.Icons.Function;

              input SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
              input SI.Density rho_a "Density at port_a";
              input SI.Density rho_b "Density at port_b";
              input SI.DynamicViscosity mu_a
                "Dynamic viscosity at port_a (dummy if use_mu = false)";
              input SI.DynamicViscosity mu_b
                "Dynamic viscosity at port_b (dummy if use_mu = false)";
              input SI.Length length "Length of pipe";
              input SI.Diameter diameter "Inner (hydraulic) diameter of pipe";
              input Real g_times_height_ab
                "Gravity times (Height(port_b) - Height(port_a))";
              input SI.Area crossArea = pi*diameter^2/4 "Inner cross section area";
              input SI.Length roughness(min=0) = 2.5e-5
                "Absolute roughness of pipe, with a default for a smooth steel pipe (dummy if use_roughness = false)";
              input SI.MassFlowRate m_flow_small = 0.01
                "Regularization of zero flow if |m_flow| < m_flow_small (dummy if use_m_flow_small = false)";
              input SI.ReynoldsNumber Re_turbulent = 4000
                "Turbulent flow if Re >= Re_turbulent (dummy if use_Re_turbulent = false)";

              output SI.Pressure dp "Pressure loss (dp = port_a.p - port_b.p)";

            annotation (Documentation(info="<html>

</html>"));
            end pressureLoss_m_flow_staticHead;
            annotation (Documentation(info="<html>

</html>"));
          end PartialWallFriction;

          package Detailed "Pipe wall friction for laminar and turbulent flow (detailed characteristic)"
            extends PartialWallFriction(
                      final use_mu = true,
                      final use_roughness = true,
                      final use_dp_small = true,
                      final use_m_flow_small = true,
                      final use_Re_turbulent = true);
            import ln = Modelica.Math.log "Logarithm, base e";
            import Modelica.Math.log10 "Logarithm, base 10";
            import Modelica.Math.exp "Exponential function";

            redeclare function extends massFlowRate_dp
              "Return mass flow rate m_flow as function of pressure loss dp, i.e., m_flow = f(dp), due to wall friction"
              import Modelica.Math;
            protected
              Real Delta = roughness/diameter "Relative roughness";
              SI.ReynoldsNumber Re1 = min((745*Math.exp(if Delta <= 0.0065 then 1 else 0.0065/Delta))^0.97, Re_turbulent)
                "Re leaving laminar curve";
              SI.ReynoldsNumber Re2 = Re_turbulent "Re entering turbulent curve";
              SI.DynamicViscosity mu "Upstream viscosity";
              SI.Density rho "Upstream density";
              SI.ReynoldsNumber Re "Reynolds number";
              Real lambda2 "Modified friction coefficient (= lambda*Re^2)";

              function interpolateInRegion2
                 input Real Re_turbulent;
                 input SI.ReynoldsNumber Re1;
                 input SI.ReynoldsNumber Re2;
                 input Real Delta;
                 input Real lambda2;
                 output SI.ReynoldsNumber Re;
                // point lg(lambda2(Re1)) with derivative at lg(Re1)
              protected
                Real x1=Math.log10(64*Re1);
                Real y1=Math.log10(Re1);
                Real yd1=1;

                // Point lg(lambda2(Re2)) with derivative at lg(Re2)
                Real aux1=(0.5/Math.log(10))*5.74*0.9;
                Real aux2=Delta/3.7 + 5.74/Re2^0.9;
                Real aux3=Math.log10(aux2);
                Real L2=0.25*(Re2/aux3)^2;
                Real aux4=2.51/sqrt(L2) + 0.27*Delta;
                Real aux5=-2*sqrt(L2)*Math.log10(aux4);
                Real x2=Math.log10(L2);
                Real y2=Math.log10(aux5);
                Real yd2=0.5 + (2.51/Math.log(10))/(aux5*aux4);

                // Constants: Cubic polynomial between lg(Re1) and lg(Re2)
                Real diff_x=x2 - x1;
                Real m=(y2 - y1)/diff_x;
                Real c2=(3*m - 2*yd1 - yd2)/diff_x;
                Real c3=(yd1 + yd2 - 2*m)/(diff_x*diff_x);
                Real lambda2_1=64*Re1;
                Real dx;
              algorithm
                 dx := Math.log10(lambda2/lambda2_1);
                 Re := Re1*(lambda2/lambda2_1)^(1 + dx*(c2 + dx*c3));
                 annotation(smoothOrder=1);
              end interpolateInRegion2;

            algorithm
              // Determine upstream density, upstream viscosity, and lambda2
              rho     := if dp >= 0 then rho_a else rho_b;
              mu      := if dp >= 0 then mu_a else mu_b;
              lambda2 := abs(dp)*2*diameter^3*rho/(length*mu*mu);

              // Determine Re under the assumption of laminar flow
              Re := lambda2/64;

              // Modify Re, if turbulent flow
              if Re > Re1 then
                 Re :=-2*sqrt(lambda2)*Math.log10(2.51/sqrt(lambda2) + 0.27*Delta);
                 if Re < Re2 then
                    Re := interpolateInRegion2(Re, Re1, Re2, Delta, lambda2);
                 end if;
              end if;

              // Determine mass flow rate
              m_flow := crossArea/diameter*mu*(if dp >= 0 then Re else -Re);
                      annotation (smoothOrder=1, Documentation(info="<html>

</html>"));
            end massFlowRate_dp;

            redeclare function extends pressureLoss_m_flow
              "Return pressure loss dp as function of mass flow rate m_flow, i.e., dp = f(m_flow), due to wall friction"
              import Modelica.Math;
              import Modelica.Constants.pi;
            protected
              Real Delta = roughness/diameter "Relative roughness";
              SI.ReynoldsNumber Re1 = min(745*Math.exp(if Delta <= 0.0065 then 1 else 0.0065/Delta), Re_turbulent)
                "Re leaving laminar curve";
              SI.ReynoldsNumber Re2 = Re_turbulent "Re entering turbulent curve";
              SI.DynamicViscosity mu "Upstream viscosity";
              SI.Density rho "Upstream density";
              SI.ReynoldsNumber Re "Reynolds number";
              Real lambda2 "Modified friction coefficient (= lambda*Re^2)";

              function interpolateInRegion2
                 input SI.ReynoldsNumber Re;
                 input SI.ReynoldsNumber Re1;
                 input SI.ReynoldsNumber Re2;
                 input Real Delta;
                 output Real lambda2;
                // point lg(lambda2(Re1)) with derivative at lg(Re1)
              protected
                Real x1 = Math.log10(Re1);
                Real y1 = Math.log10(64*Re1);
                Real yd1=1;

                // Point lg(lambda2(Re2)) with derivative at lg(Re2)
                Real aux1=(0.5/Math.log(10))*5.74*0.9;
                Real aux2=Delta/3.7 + 5.74/Re2^0.9;
                Real aux3=Math.log10(aux2);
                Real L2=0.25*(Re2/aux3)^2;
                Real aux4=2.51/sqrt(L2) + 0.27*Delta;
                Real aux5=-2*sqrt(L2)*Math.log10(aux4);
                Real x2 =  Math.log10(Re2);
                Real y2 =  Math.log10(L2);
                Real yd2 = 2 + 4*aux1/(aux2*aux3*(Re2)^0.9);

                // Constants: Cubic polynomial between lg(Re1) and lg(Re2)
                Real diff_x=x2 - x1;
                Real m=(y2 - y1)/diff_x;
                Real c2=(3*m - 2*yd1 - yd2)/diff_x;
                Real c3=(yd1 + yd2 - 2*m)/(diff_x*diff_x);
                Real dx;
              algorithm
                 dx := Math.log10(Re/Re1);
                 lambda2 := 64*Re1*(Re/Re1)^(1 + dx*(c2 + dx*c3));
                 annotation(smoothOrder=1);
              end interpolateInRegion2;
            algorithm
              // Determine upstream density and upstream viscosity
              rho     :=if m_flow >= 0 then rho_a else rho_b;
              mu      :=if m_flow >= 0 then mu_a else mu_b;

              // Determine Re, lambda2 and pressure drop
              Re := diameter*abs(m_flow)/(crossArea*mu);
              lambda2 := if Re <= Re1 then 64*Re else
                        (if Re >= Re2 then 0.25*(Re/Math.log10(Delta/3.7 + 5.74/Re^0.9))^2 else
                         interpolateInRegion2(Re, Re1, Re2, Delta));
              dp :=length*mu*mu/(2*rho*diameter*diameter*diameter)*
                   (if m_flow >= 0 then lambda2 else -lambda2);
                      annotation (smoothOrder=1, Documentation(info="<html>

</html>"));
            end pressureLoss_m_flow;

            redeclare function extends massFlowRate_dp_staticHead
              "Return mass flow rate m_flow as function of pressure loss dp, i.e., m_flow = f(dp), due to wall friction and static head"

            protected
              Real Delta = roughness/diameter "Relative roughness";
              SI.ReynoldsNumber Re "Reynolds number";
              SI.ReynoldsNumber Re1 = min((745*exp(if Delta <= 0.0065 then 1 else 0.0065/Delta))^0.97, Re_turbulent)
                "Boundary between laminar regime and transition";
              SI.ReynoldsNumber Re2 = Re_turbulent
                "Boundary between transition and turbulent regime";
              SI.Pressure dp_a
                "Upper end of regularization domain of the m_flow(dp) relation";
              SI.Pressure dp_b
                "Lower end of regularization domain of the m_flow(dp) relation";
              SI.MassFlowRate m_flow_a
                "Value at upper end of regularization domain";
              SI.MassFlowRate m_flow_b
                "Value at lower end of regularization domain";

              SI.MassFlowRate dm_flow_ddp_fric_a
                "Derivative at upper end of regularization domain";
              SI.MassFlowRate dm_flow_ddp_fric_b
                "Derivative at lower end of regularization domain";

              SI.Pressure dp_grav_a = g_times_height_ab*rho_a
                "Static head if mass flows in design direction (a to b)";
              SI.Pressure dp_grav_b = g_times_height_ab*rho_b
                "Static head if mass flows against design direction (b to a)";

              // Properly define zero mass flow conditions
              SI.MassFlowRate m_flow_zero = 0;
              SI.Pressure dp_zero = (dp_grav_a + dp_grav_b)/2;
              Real dm_flow_ddp_fric_zero;

            algorithm
              dp_a := max(dp_grav_a, dp_grav_b)+dp_small;
              dp_b := min(dp_grav_a, dp_grav_b)-dp_small;

              if dp>=dp_a then
                // Positive flow outside regularization
                m_flow := Internal.m_flow_of_dp_fric(dp-dp_grav_a, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta);
              elseif dp<=dp_b then
                // Negative flow outside regularization
                m_flow := Internal.m_flow_of_dp_fric(dp-dp_grav_b, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta);
              else
                // Regularization parameters
                (m_flow_a, dm_flow_ddp_fric_a) := Internal.m_flow_of_dp_fric(dp_a-dp_grav_a, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta);
                (m_flow_b, dm_flow_ddp_fric_b) := Internal.m_flow_of_dp_fric(dp_b-dp_grav_b, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta);
                // Include a properly defined zero mass flow point
                // Obtain a suitable slope from the linear section slope c (value of m_flow is overwritten later)
                (m_flow, dm_flow_ddp_fric_zero) := Utilities.regFun3(dp_zero, dp_b, dp_a, m_flow_b, m_flow_a, dm_flow_ddp_fric_b, dm_flow_ddp_fric_a);
                // Do regularization
                if dp>dp_zero then
                  m_flow := Utilities.regFun3(dp, dp_zero, dp_a, m_flow_zero, m_flow_a, dm_flow_ddp_fric_zero, dm_flow_ddp_fric_a);
                else
                  m_flow := Utilities.regFun3(dp, dp_b, dp_zero, m_flow_b, m_flow_zero, dm_flow_ddp_fric_b, dm_flow_ddp_fric_zero);
                end if;
              end if;
              annotation (smoothOrder=1);
            end massFlowRate_dp_staticHead;

            redeclare function extends pressureLoss_m_flow_staticHead
              "Return pressure loss dp as function of mass flow rate m_flow, i.e., dp = f(m_flow), due to wall friction and static head"

            protected
              Real Delta = roughness/diameter "Relative roughness";
              SI.ReynoldsNumber Re1 = min(745*exp(if Delta <= 0.0065 then 1 else 0.0065/Delta), Re_turbulent)
                "Boundary between laminar regime and transition";
              SI.ReynoldsNumber Re2 = Re_turbulent
                "Boundary between transition and turbulent regime";

              SI.MassFlowRate m_flow_a
                "Upper end of regularization domain of the dp(m_flow) relation";
              SI.MassFlowRate m_flow_b
                "Lower end of regularization domain of the dp(m_flow) relation";

              SI.Pressure dp_a "Value at upper end of regularization domain";
              SI.Pressure dp_b "Value at lower end of regularization domain";

              SI.Pressure dp_grav_a = g_times_height_ab*rho_a
                "Static head if mass flows in design direction (a to b)";
              SI.Pressure dp_grav_b = g_times_height_ab*rho_b
                "Static head if mass flows against design direction (b to a)";

              Real ddp_dm_flow_a
                "Derivative of pressure drop with mass flow rate at m_flow_a";
              Real ddp_dm_flow_b
                "Derivative of pressure drop with mass flow rate at m_flow_b";

              // Properly define zero mass flow conditions
              SI.MassFlowRate m_flow_zero = 0;
              SI.Pressure dp_zero = (dp_grav_a + dp_grav_b)/2;
              Real ddp_dm_flow_zero;

            algorithm
              m_flow_a := if dp_grav_a<dp_grav_b then
                Internal.m_flow_of_dp_fric(dp_grav_b - dp_grav_a, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta)+m_flow_small else
                m_flow_small;
              m_flow_b := if dp_grav_a<dp_grav_b then
                Internal.m_flow_of_dp_fric(dp_grav_a - dp_grav_b, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta)-m_flow_small else
                -m_flow_small;

              if m_flow>=m_flow_a then
                // Positive flow outside regularization
                dp := Internal.dp_fric_of_m_flow(m_flow, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta) + dp_grav_a;
              elseif m_flow<=m_flow_b then
                // Negative flow outside regularization
                dp := Internal.dp_fric_of_m_flow(m_flow, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta) + dp_grav_b;
              else
                // Regularization parameters
                (dp_a, ddp_dm_flow_a) := Internal.dp_fric_of_m_flow(m_flow_a, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta);
                dp_a := dp_a + dp_grav_a "Adding dp_grav to dp_fric to get dp";
                (dp_b, ddp_dm_flow_b) := Internal.dp_fric_of_m_flow(m_flow_b, rho_a, rho_b, mu_a, mu_b, length, diameter, crossArea, Re1, Re2, Delta);
                dp_b := dp_b + dp_grav_b "Adding dp_grav to dp_fric to get dp";
                // Include a properly defined zero mass flow point
                // Obtain a suitable slope from the linear section slope c (value of dp is overwritten later)
                (dp, ddp_dm_flow_zero) := Utilities.regFun3(m_flow_zero, m_flow_b, m_flow_a, dp_b, dp_a, ddp_dm_flow_b, ddp_dm_flow_a);
                // Do regularization
                if m_flow>m_flow_zero then
                  dp := Utilities.regFun3(m_flow, m_flow_zero, m_flow_a, dp_zero, dp_a, ddp_dm_flow_zero, ddp_dm_flow_a);
                else
                  dp := Utilities.regFun3(m_flow, m_flow_b, m_flow_zero, dp_b, dp_zero, ddp_dm_flow_b, ddp_dm_flow_zero);
                end if;
              end if;
              annotation (smoothOrder=1);
            end pressureLoss_m_flow_staticHead;

          package Internal "Functions to calculate mass flow rate from friction pressure drop and vice versa"
            extends Modelica.Icons.InternalPackage;

            function m_flow_of_dp_fric
                "Calculate mass flow rate as function of pressure drop due to friction"
              extends Modelica.Icons.Function;

              input SI.Pressure dp_fric
                  "Pressure loss due to friction (dp = port_a.p - port_b.p)";
              input SI.Density rho_a "Density at port_a";
              input SI.Density rho_b "Density at port_b";
              input SI.DynamicViscosity mu_a
                  "Dynamic viscosity at port_a (dummy if use_mu = false)";
              input SI.DynamicViscosity mu_b
                  "Dynamic viscosity at port_b (dummy if use_mu = false)";
              input SI.Length length "Length of pipe";
              input SI.Diameter diameter "Inner (hydraulic) diameter of pipe";
              input SI.Area crossArea "Inner cross section area";
              input SI.ReynoldsNumber Re1
                  "Boundary between laminar regime and transition";
              input SI.ReynoldsNumber Re2
                  "Boundary between transition and turbulent regime";
              input Real Delta "Relative roughness";
              output SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
              output Real dm_flow_ddp_fric
                  "Derivative of mass flow rate with dp_fric";

              protected
              function interpolateInRegion2_withDerivative
                  "Interpolation in log-log space using a cubic Hermite polynomial, where x=log10(lambda2), y=log10(Re)"
                extends Modelica.Icons.Function;

                input Real lambda2 "Known independent variable";
                input SI.ReynoldsNumber Re1
                    "Boundary between laminar regime and transition";
                input SI.ReynoldsNumber Re2
                    "Boundary between transition and turbulent regime";
                input Real Delta "Relative roughness";
                input SI.Pressure dp_fric
                    "Pressure loss due to friction (dp = port_a.p - port_b.p)";
                output SI.ReynoldsNumber Re "Unknown return variable";
                output Real dRe_ddp "Derivative of return value";
                // point lg(lambda2(Re1)) with derivative at lg(Re1)
                protected
                Real x1=log10(64*Re1);
                Real y1=log10(Re1);
                Real y1d=1;

                // Point lg(lambda2(Re2)) with derivative at lg(Re2)
                Real aux2=Delta/3.7 + 5.74/Re2^0.9;
                Real aux3=log10(aux2);
                Real L2=0.25*(Re2/aux3)^2;
                Real aux4=2.51/sqrt(L2) + 0.27*Delta;
                Real aux5=-2*sqrt(L2)*log10(aux4);
                Real x2=log10(L2);
                Real y2=log10(aux5);
                Real y2d=0.5 + (2.51/log(10))/(aux5*aux4);

                // Point of interest in transformed space
                Real x=log10(lambda2);
                Real y;
                Real dy_dx "Derivative in transformed space";
              algorithm
                // Interpolation
                (y, dy_dx) := Utilities.cubicHermite_withDerivative(x, x1, x2, y1, y2, y1d, y2d);

                // Return value
                Re := 10^y;

                // Derivative of return value
                dRe_ddp := Re/abs(dp_fric)*dy_dx;
                annotation (smoothOrder=1);
              end interpolateInRegion2_withDerivative;

              SI.DynamicViscosity mu "Upstream viscosity";
              SI.Density rho "Upstream density";
              Real lambda2 "Modified friction coefficient (= lambda*Re^2)";
              SI.ReynoldsNumber Re "Reynolds number";
              Real dRe_ddp "dRe/ddp";
              Real aux1;
              Real aux2;

            algorithm
              // Determine upstream density and upstream viscosity
              if dp_fric >= 0 then
                rho := rho_a;
                mu  := mu_a;
              else
                rho := rho_b;
                mu  := mu_b;
              end if;

              // Positive mass flow rate
              lambda2 := abs(dp_fric)*2*diameter^3*rho/(length*mu*mu)
                  "Known as lambda2=f(dp)";

              aux1:=(2*diameter^3*rho)/(length*mu^2);

              // Determine Re and dRe/ddp under the assumption of laminar flow
              Re := lambda2/64 "Hagen-Poiseuille";
              dRe_ddp := aux1/64 "Hagen-Poiseuille";

              // Modify Re, if turbulent flow
              if Re > Re1 then
                Re :=-2*sqrt(lambda2)*log10(2.51/sqrt(lambda2) + 0.27*Delta)
                    "Colebrook-White";
                aux2 := sqrt(aux1*abs(dp_fric));
                dRe_ddp := 1/log(10)*(-2*log(2.51/aux2+0.27*Delta)*aux1/(2*aux2)+2*2.51/(2*abs(dp_fric)*(2.51/aux2+0.27*Delta)));
                if Re < Re2 then
                  (Re, dRe_ddp) := interpolateInRegion2_withDerivative(lambda2, Re1, Re2, Delta, dp_fric);
                end if;
              end if;

              // Determine mass flow rate
              m_flow := crossArea/diameter*mu*(if dp_fric >= 0 then Re else -Re);
              // Determine derivative of mass flow rate with dp_fric
              dm_flow_ddp_fric := crossArea/diameter*mu*dRe_ddp;
              annotation(smoothOrder=1);
            end m_flow_of_dp_fric;

            function dp_fric_of_m_flow
                "Calculate pressure drop due to friction as function of mass flow rate"
              extends Modelica.Icons.Function;

              input SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
              input SI.Density rho_a "Density at port_a";
              input SI.Density rho_b "Density at port_b";
              input SI.DynamicViscosity mu_a
                  "Dynamic viscosity at port_a (dummy if use_mu = false)";
              input SI.DynamicViscosity mu_b
                  "Dynamic viscosity at port_b (dummy if use_mu = false)";
              input SI.Length length "Length of pipe";
              input SI.Diameter diameter "Inner (hydraulic) diameter of pipe";
              input SI.Area crossArea "Inner cross section area";
              input SI.ReynoldsNumber Re1
                  "Boundary between laminar regime and transition";
              input SI.ReynoldsNumber Re2
                  "Boundary between transition and turbulent regime";
              input Real Delta "Relative roughness";
              output SI.Pressure dp_fric
                  "Pressure loss due to friction (dp_fric = port_a.p - port_b.p - dp_grav)";
              output Real ddp_fric_dm_flow
                  "Derivative of pressure drop with mass flow rate";

              protected
              function interpolateInRegion2
                  "Interpolation in log-log space using a cubic Hermite polynomial, where x=log10(Re), y=log10(lambda2)"
                extends Modelica.Icons.Function;

                input SI.ReynoldsNumber Re "Known independent variable";
                input SI.ReynoldsNumber Re1
                    "Boundary between laminar regime and transition";
                input SI.ReynoldsNumber Re2
                    "Boundary between transition and turbulent regime";
                input Real Delta "Relative roughness";
                input SI.MassFlowRate m_flow "Mass flow rate from port_a to port_b";
                output Real lambda2 "Unknown return value";
                output Real dlambda2_dm_flow "Derivative of return value";
                // point lg(lambda2(Re1)) with derivative at lg(Re1)
                protected
                Real x1 = log10(Re1);
                Real y1 = log10(64*Re1);
                Real y1d = 1;

                // Point lg(lambda2(Re2)) with derivative at lg(Re2)
                Real aux2 = Delta/3.7 + 5.74/Re2^0.9;
                Real aux3 = log10(aux2);
                Real L2 = 0.25*(Re2/aux3)^2;
                Real x2 = log10(Re2);
                Real y2 = log10(L2);
                Real y2d = 2+(2*5.74*0.9)/(log(aux2)*Re2^0.9*aux2);

                // Point of interest in transformed space
                Real x=log10(Re);
                Real y;
                Real dy_dx "Derivative in transformed space";
              algorithm
                // Interpolation
                (y, dy_dx) := Utilities.cubicHermite_withDerivative(x, x1, x2, y1, y2, y1d, y2d);

                // Return value
                lambda2 := 10^y;

                // Derivative of return value
                dlambda2_dm_flow := lambda2/abs(m_flow)*dy_dx;
                annotation(smoothOrder=1);
              end interpolateInRegion2;

              SI.DynamicViscosity mu "Upstream viscosity";
              SI.Density rho "Upstream density";
              SI.ReynoldsNumber Re "Reynolds number";
              Real lambda2 "Modified friction coefficient (= lambda*Re^2)";
              Real dlambda2_dm_flow "dlambda2/dm_flow";
              Real aux1;
              Real aux2;

            algorithm
              // Determine upstream density and upstream viscosity
              if m_flow >= 0 then
                rho := rho_a;
                mu  := mu_a;
              else
                rho := rho_b;
                mu  := mu_b;
              end if;

              // Determine Reynolds number
              Re := abs(m_flow)*diameter/(crossArea*mu);

              aux1 := diameter/(crossArea*mu);

              // Use correlation for lambda2 depending on actual conditions
              if Re <= Re1 then
                lambda2 := 64*Re "Hagen-Poiseuille";
                dlambda2_dm_flow := 64*aux1 "Hagen-Poiseuille";
              elseif Re >= Re2 then
                lambda2 := 0.25*(Re/log10(Delta/3.7 + 5.74/Re^0.9))^2 "Swamee-Jain";
                aux2 := Delta/3.7+5.74/((aux1*abs(m_flow))^0.9);
                dlambda2_dm_flow := 0.5*aux1*Re*log(10)^2*(1/(log(aux2)^2)+(5.74*0.9)/(log(aux2)^3*Re^0.9*aux2))
                    "Swamee-Jain";
              else
                (lambda2, dlambda2_dm_flow) := interpolateInRegion2(Re, Re1, Re2, Delta, m_flow);
              end if;

              // Compute pressure drop from lambda2
              dp_fric :=length*mu*mu/(2*rho*diameter*diameter*diameter)*
                   (if m_flow >= 0 then lambda2 else -lambda2);

              // Compute derivative from dlambda2/dm_flow
              ddp_fric_dm_flow := (length*mu^2)/(2*diameter^3*rho)*dlambda2_dm_flow;
              annotation(smoothOrder=1);
            end dp_fric_of_m_flow;
          end Internal;
            annotation (Documentation(info="<html>
<p>
This component defines the complete regime of wall friction.
The details are described in the
<a href=\"modelica://Modelica.Fluid.UsersGuide.ComponentDefinition.WallFriction\">UsersGuide</a>.
The functional relationship of the friction loss factor &lambda; is
displayed in the next figure. Function massFlowRate_dp() defines the \"red curve\"
(\"Swamee and Jain\"), where as function pressureLoss_m_flow() defines the
\"blue curve\" (\"Colebrook-White\"). The two functions are inverses from
each other and give slightly different results in the transition region
between Re = 1500 .. 4000, in order to get explicit equations without
solving a non-linear equation.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/PipeFriction1.png\"
     alt=\"PipeFriction1.png\">
</p>

<p>
Additionally to wall friction, this component properly implements static
head. With respect to the latter, two cases can be distinguished. In the case
shown next, the change of elevation with the path from a to b has the opposite
sign of the change of density.</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/PipeFrictionStaticHead_case-a.png\"
     alt=\"PipeFrictionStaticHead_case-a.png\">
</p>

<p>
In the case illustrated second, the change of elevation with the path from a to
b has the same sign of the change of density.</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/PipeFrictionStaticHead_case-b.png\"
     alt=\"PipeFrictionStaticHead_case-b.png\">
</p>

</html>"));
          end Detailed;
          annotation (Documentation(info="<html>
<p>
This package provides functions to compute
pressure losses due to <strong>wall friction</strong> in a pipe.
Every correlation is defined by a package that is derived
by inheritance from the package WallFriction.PartialWallFriction.
The details of the underlying pipe wall friction model are described in the
<a href=\"modelica://Modelica.Fluid.UsersGuide.ComponentDefinition.WallFriction\">UsersGuide</a>.
Basically, different variants of the equation
</p>

<pre>
   dp = &lambda;(Re,<font face=\"Symbol\">D</font>)*(L/D)*&rho;*v*|v|/2
</pre>

<p>
are used, where the friction loss factor &lambda; is shown
in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/PipeFriction1.png\"
     alt=\"PipeFriction1.png\">
</p>

</html>"));
        end WallFriction;
      end BaseClasses;
      annotation (Documentation(info="<html>

</html>"));
    end Pipes;

    package Sensors "Ideal sensor components to extract signals from a fluid connector"
      extends Modelica.Icons.SensorsPackage;

      model Pressure "Ideal pressure sensor"
        extends Sensors.BaseClasses.PartialAbsoluteSensor;
        extends Modelica.Icons.RotationalSensor;
        Modelica.Blocks.Interfaces.RealOutput p(final quantity="Pressure",
                                                final unit="Pa",
                                                displayUnit="bar",
                                                min=0) "Pressure at port"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        p = port.p;
        annotation (
        Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{70,0},{100,0}}, color={0,0,127}),
              Line(points={{0,-70},{0,-100}}, color={0,127,255}),
              Text(
                extent={{-150,80},{150,120}},
                textString="%name",
                lineColor={0,0,255}),
              Text(
                extent={{151,-20},{57,-50}},
                textString="p")}),
          Documentation(info="<html>
<p>
This component monitors the absolute pressure at its fluid port. The sensor is
ideal, i.e., it does not influence the fluid.
</p>
</html>"));
      end Pressure;

      model SpecificEnthalpy "Ideal one port specific enthalpy sensor"
        extends Sensors.BaseClasses.PartialAbsoluteSensor;
        extends Modelica.Icons.RotationalSensor;
        Modelica.Blocks.Interfaces.RealOutput h_out(final quantity="SpecificEnergy",
                                                    final unit="J/kg")
          "Specific enthalpy in port medium"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      equation
        h_out = inStream(port.h_outflow);
      annotation (defaultComponentName="specificEnthalpy",
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics={
              Line(points={{0,-70},{0,-100}}, color={0,0,127}),
              Text(
                extent={{-150,80},{150,120}},
                textString="%name",
                lineColor={0,0,255}),
              Text(
                extent={{168,-30},{52,-60}},
                textString="h"),
              Line(points={{70,0},{100,0}}, color={0,0,127})}),
        Documentation(info="<html>
<p>
This component monitors the specific enthalpy of the fluid passing its port.
The sensor is ideal, i.e., it does not influence the fluid.
</p>
</html>"));
      end SpecificEnthalpy;

      model MassFlowRate "Ideal sensor for mass flow rate"
        extends Sensors.BaseClasses.PartialFlowSensor;
        extends Modelica.Icons.RotationalSensor;
        Modelica.Blocks.Interfaces.RealOutput m_flow(quantity="MassFlowRate",
                                                     final unit="kg/s")
          "Mass flow rate from port_a to port_b" annotation (Placement(
              transformation(
              origin={0,110},
              extent={{10,-10},{-10,10}},
              rotation=270)));

      equation
        m_flow = port_a.m_flow;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics={
              Line(points={{70,0},{100,0}}, color={0,128,255}),
              Text(
                extent={{162,120},{2,90}},
                textString="m_flow"),
              Line(points={{0,100},{0,70}}, color={0,0,127}),
              Line(points={{-100,0},{-70,0}}, color={0,128,255})}),
        Documentation(info="<html>
<p>
This component monitors the mass flow rate flowing from port_a to port_b.
The sensor is ideal, i.e., it does not influence the fluid.
</p>
</html>"));
      end MassFlowRate;

      model VolumeFlowRate "Ideal sensor for volume flow rate"
        extends Sensors.BaseClasses.PartialFlowSensor;
        extends Modelica.Icons.RotationalSensor;
        Modelica.Blocks.Interfaces.RealOutput V_flow(final quantity="VolumeFlowRate",
                                                     final unit="m3/s")
          "Volume flow rate from port_a to port_b"
          annotation (Placement(transformation(
              origin={0,110},
              extent={{10,-10},{-10,10}},
              rotation=270)));

      protected
        Medium.Density rho_a_inflow "Density of inflowing fluid at port_a";
        Medium.Density rho_b_inflow
          "Density of inflowing fluid at port_b or rho_a_inflow, if uni-directional flow";
        Medium.Density d "Density of the passing fluid";
      equation
        if allowFlowReversal then
           rho_a_inflow = Medium.density(Medium.setState_phX(port_b.p, port_b.h_outflow, port_b.Xi_outflow));
           rho_b_inflow = Medium.density(Medium.setState_phX(port_a.p, port_a.h_outflow, port_a.Xi_outflow));
           d = Modelica.Fluid.Utilities.regStep(port_a.m_flow, rho_a_inflow, rho_b_inflow, m_flow_small);
        else
           d = Medium.density(Medium.setState_phX(port_b.p, port_b.h_outflow, port_b.Xi_outflow));
           rho_a_inflow = d;
           rho_b_inflow = d;
        end if;
        V_flow = port_a.m_flow/d;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics={
              Text(
                extent={{160,120},{0,90}},
                textString="V_flow"),
              Line(points={{0,100},{0,70}}, color={0,0,127}),
              Line(points={{-100,0},{-70,0}}, color={0,128,255}),
              Line(points={{70,0},{100,0}}, color={0,128,255})}),
        Documentation(info="<html>
<p>
This component monitors the volume flow rate flowing from port_a to port_b.
The sensor is ideal, i.e., it does not influence the fluid.
</p>
</html>"));
      end VolumeFlowRate;

      model RelativePressure "Ideal relative pressure sensor"
        extends Sensors.BaseClasses.PartialRelativeSensor;

        Modelica.Blocks.Interfaces.RealOutput p_rel(final quantity="Pressure",
                                                    final unit="Pa",
                                                    displayUnit="bar")
          "Relative pressure signal" annotation (Placement(transformation(
              origin={0,-90},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation

        // Relative pressure
        p_rel = port_a.p - port_b.p;
        annotation (
          Icon(graphics={
              Line(points={{0,-30},{0,-80}}, color={0,0,127}),
              Text(
                extent={{130,-70},{4,-100}},
                textString="p_rel")}),
          Documentation(info="<html>
<p>
The relative pressure \"port_a.p - port_b.p\" is determined between
the two ports of this component and is provided as output signal. The
sensor should be connected in parallel with other equipment, no flow
through the sensor is allowed.
</p>
</html>"));
      end RelativePressure;

      package BaseClasses "Base classes used in the Sensors package (only of interest to build new component models)"
        extends Modelica.Icons.BasesPackage;

        partial model PartialAbsoluteSensor
          "Partial component to model a sensor that measures a potential variable"

          replaceable package Medium=Modelica.Media.Interfaces.PartialMedium
            "Medium in the sensor"
            annotation(choicesAllMatching=true);

          Modelica.Fluid.Interfaces.FluidPort_a port(redeclare package Medium=Medium, m_flow(min=0))
            annotation (Placement(transformation(
                origin={0,-100},
                extent={{-10,-10},{10,10}},
                rotation=90)));

        equation
          port.m_flow = 0;
          port.h_outflow = Medium.h_default;
          port.Xi_outflow = Medium.X_default[1:Medium.nXi];
          port.C_outflow = zeros(Medium.nC);
          annotation (Documentation(info="<html>
<p>
Partial component to model an <strong>absolute sensor</strong>. Can be used for pressure sensor models.
Use for other properties such as temperature or density is discouraged, because the enthalpy at the connector can have different meanings, depending on the connection topology. Use <code>PartialFlowSensor</code> instead.
as signal.
</p>
</html>"));
        end PartialAbsoluteSensor;

        model PartialRelativeSensor
          "Partial component to model a sensor that measures the difference between two potential variables"
          extends Modelica.Icons.TranslationalSensor;
          replaceable package Medium =
            Modelica.Media.Interfaces.PartialMedium "Medium in the sensor" annotation (
              choicesAllMatching = true);

          Modelica.Fluid.Interfaces.FluidPort_a port_a(m_flow(min=0),
                                        redeclare package Medium = Medium)
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
          Modelica.Fluid.Interfaces.FluidPort_b port_b(m_flow(min=0),
                                        redeclare package Medium = Medium)
            annotation (Placement(transformation(extent={{110,-12},{90,8}}), iconTransformation(extent={{110,-10},{90,10}})));

        equation
          // Zero flow equations for connectors
          port_a.m_flow = 0;
          port_b.m_flow = 0;

          // No contribution of specific quantities
          port_a.h_outflow = Medium.h_default;
          port_b.h_outflow = Medium.h_default;
          port_a.Xi_outflow = Medium.X_default[1:Medium.nXi];
          port_b.Xi_outflow = Medium.X_default[1:Medium.nXi];
          port_a.C_outflow  = zeros(Medium.nC);
          port_b.C_outflow  = zeros(Medium.nC);

          annotation (
            Icon(graphics={
                Line(points={{-100,0},{-70,0}}, color={0,127,255}),
                Line(points={{70,0},{100,0}}, color={0,127,255}),
                Text(
                  extent={{-150,40},{150,80}},
                  textString="%name",
                  lineColor={0,0,255}),
                Line(
                  points={{32,3},{-58,3}},
                  color={0,128,255}),
                Polygon(
                  points={{22,18},{62,3},{22,-12},{22,18}},
                  lineColor={0,128,255},
                  fillColor={0,128,255},
                  fillPattern=FillPattern.Solid)}),
            Documentation(info="<html>
<p>
The relative pressure \"port_a.p - port_b.p\" is determined between
the two ports of this component and is provided as output signal. The
sensor should be connected in parallel with other equipment, no flow
through the sensor is allowed.
</p>
</html>"));
        end PartialRelativeSensor;

        partial model PartialFlowSensor
          "Partial component to model sensors that measure flow properties"
          extends Modelica.Fluid.Interfaces.PartialTwoPort;

          parameter Medium.MassFlowRate m_flow_nominal = system.m_flow_nominal
            "Nominal value of m_flow = port_a.m_flow"
            annotation(Dialog(tab = "Advanced"));
          parameter Medium.MassFlowRate m_flow_small(min=0) = if system.use_eps_Re then system.eps_m_flow*m_flow_nominal else system.m_flow_small
            "Regularization for bi-directional flow in the region |m_flow| < m_flow_small (m_flow_small > 0 required)"
            annotation(Dialog(tab="Advanced"));

        equation
          // mass balance
          0 = port_a.m_flow + port_b.m_flow;

          // momentum equation (no pressure loss)
          port_a.p = port_b.p;

          // isenthalpic state transformation (no storage and no loss of energy)
          port_a.h_outflow = inStream(port_b.h_outflow);
          port_b.h_outflow = inStream(port_a.h_outflow);

          port_a.Xi_outflow = inStream(port_b.Xi_outflow);
          port_b.Xi_outflow = inStream(port_a.Xi_outflow);

          port_a.C_outflow = inStream(port_b.C_outflow);
          port_b.C_outflow = inStream(port_a.C_outflow);
          annotation (Documentation(info="<html>
<p>
Partial component to model a <strong>sensor</strong> that measures any intensive properties
of a flow, e.g., to get temperature or density in the flow
between fluid connectors.<br>
The model includes zero-volume balance equations. Sensor models inheriting from
this partial class should add a medium instance to calculate the measured property.
</p>
</html>"));
        end PartialFlowSensor;
      end BaseClasses;
      annotation (preferredView="info", Documentation(info="<html>
<p>
Package <strong>Sensors</strong> consists of idealized sensor components that
provide variables of a medium model and/or fluid ports as
output signals. These signals can be, e.g., further processed
with components of the Modelica.Blocks library.
Also more realistic sensor models can be built, by further
processing (e.g., by attaching block Modelica.Blocks.FirstOrder to
model the time constant of the sensor).
</p>

<p>For the thermodynamic state variables temperature, specific enthalpy, specific entropy and density
the fluid library provides two different types of sensors: <strong>regular one port</strong> and <strong>two port</strong> sensors.</p>

<ul>
<li>The <strong>regular one port</strong> sensors have the advantage of easy introduction and removal from a model, as no connections have to be broken.
A potential drawback is that the obtained value jumps as flow reverts.
</li>

<li>The <strong>two port</strong> sensors offer the advantages of an adjustable regularized step function around zero flow.
Moreover the obtained result is restricted to the value flowing into port_a if allowFlowReversal is false.</li>
</ul>

<p>
<a href=\"modelica://Modelica.Fluid.Examples.Explanatory.MeasuringTemperature\">Modelica.Fluid.Examples.Explanatory.MeasuringTemperature</a>
demonstrates the differences between one- and two-port sensor at hand of a
simple example.
</p>
</html>",     revisions="<html>
<ul>
<li><em>22 Dec 2008</em>
    by R;uumldiger Franke<br>
    <ul>
    <li>flow sensors based on Interfaces.PartialTwoPort</li>
    <li>adapted docu to stream connectors, i.e., less need for two port sensors</li>
    </ul>
<li><em>4 Dec 2008</em>
    by Michael Wetter<br>
       included sensors for trace substance</li>
<li><em>31 Oct 2007</em>
    by Carsten Heinrich<br>
       updated sensor models, included one and two port sensors for thermodynamic state variables</li>
</ul>
</html>"));
    end Sensors;

    package Interfaces "Interfaces for steady state and unsteady, mixed-phase, multi-substance, incompressible and compressible flow"
      extends Modelica.Icons.InterfacesPackage;

      connector FluidPort
        "Interface for quasi one-dimensional fluid flow in a piping network (incompressible or compressible, one or more phases, one or more substances)"

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);

        flow Medium.MassFlowRate m_flow
          "Mass flow rate from the connection point into the component";
        Medium.AbsolutePressure p "Thermodynamic pressure in the connection point";
        stream Medium.SpecificEnthalpy h_outflow
          "Specific thermodynamic enthalpy close to the connection point if m_flow < 0";
        stream Medium.MassFraction Xi_outflow[Medium.nXi]
          "Independent mixture mass fractions m_i/m close to the connection point if m_flow < 0";
        stream Medium.ExtraProperty C_outflow[Medium.nC]
          "Properties c_i/m close to the connection point if m_flow < 0";
      end FluidPort;

      connector FluidPort_a "Generic fluid connector at design inlet"
        extends FluidPort;
        annotation (defaultComponentName="port_a",
                    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={Ellipse(
                extent={{-40,40},{40,-40}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid), Text(extent={{-150,110},{150,50}},
                  textString="%name")}),
             Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}), graphics={Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,127,255},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid), Ellipse(
                extent={{-100,100},{100,-100}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid)}));
      end FluidPort_a;

      connector FluidPort_b "Generic fluid connector at design outlet"
        extends FluidPort;
        annotation (defaultComponentName="port_b",
                    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={
              Ellipse(
                extent={{-40,40},{40,-40}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,30},{30,-30}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(extent={{-150,110},{150,50}}, textString="%name")}),
             Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,127,255},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-100,100},{100,-100}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-80,80},{80,-80}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FluidPort_b;

      connector FluidPorts_b
        "Fluid connector with outlined, large icon to be used for vectors of FluidPorts (vector dimensions must be added after dragging)"
        extends FluidPort;
        annotation (defaultComponentName="ports_b",
                    Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-50,-200},{50,200}},
              initialScale=0.2), graphics={
              Text(extent={{-75,130},{75,100}}, textString="%name"),
              Rectangle(
                extent={{-25,100},{25,-100}}),
              Ellipse(
                extent={{-25,90},{25,40}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-25,25},{25,-25}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-25,-40},{25,-90}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-15,-50},{15,-80}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-15,15},{15,-15}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-15,50},{15,80}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}),
             Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-50,-200},{50,200}},
              initialScale=0.2), graphics={
              Rectangle(
                extent={{-50,200},{50,-200}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,180},{50,80}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,50},{50,-50}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,-80},{50,-180}},
                fillColor={0,127,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,30},{30,-30}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,100},{30,160}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-100},{30,-160}},
                lineColor={0,127,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FluidPorts_b;

      partial model PartialTwoPort "Partial component with two ports"
        import Modelica.Constants;
        outer Modelica.Fluid.System system "System wide properties";

        replaceable package Medium =
            Modelica.Media.Interfaces.PartialMedium "Medium in the component"
            annotation (choicesAllMatching = true);

        parameter Boolean allowFlowReversal = system.allowFlowReversal
          "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
          annotation(Dialog(tab="Assumptions"), Evaluate=true);

        Modelica.Fluid.Interfaces.FluidPort_a port_a(
                                      redeclare package Medium = Medium,
                           m_flow(min=if allowFlowReversal then -Constants.inf else 0))
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(
                                      redeclare package Medium = Medium,
                           m_flow(max=if allowFlowReversal then +Constants.inf else 0))
          "Fluid connector b (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{110,-10},{90,10}}), iconTransformation(extent={{110,-10},{90,10}})));
        // Model structure, e.g., used for visualization
      protected
        parameter Boolean port_a_exposesState = false
          "= true if port_a exposes the state of a fluid volume";
        parameter Boolean port_b_exposesState = false
          "= true if port_b.p exposes the state of a fluid volume";
        parameter Boolean showDesignFlowDirection = true
          "= false to hide the arrow in the model icon";

        annotation (
          Documentation(info="<html>
<p>
This partial model defines an interface for components with two ports.
The treatment of the design flow direction and of flow reversal are predefined based on the parameter <code><strong>allowFlowReversal</strong></code>.
The component may transport fluid and may have internal storage for a given fluid <code><strong>Medium</strong></code>.
</p>
<p>
An extending model providing direct access to internal storage of mass or energy through port_a or port_b
should redefine the protected parameters <code><strong>port_a_exposesState</strong></code> and <code><strong>port_b_exposesState</strong></code> appropriately.
This will be visualized at the port icons, in order to improve the understanding of fluid model diagrams.
</p>
</html>"),Icon(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-100,-100},{100,100}}), graphics={
              Polygon(
                points={{20,-70},{60,-85},{20,-100},{20,-70}},
                lineColor={0,128,255},
                fillColor={0,128,255},
                fillPattern=FillPattern.Solid,
                visible=showDesignFlowDirection),
              Polygon(
                points={{20,-75},{50,-85},{20,-95},{20,-75}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                visible=allowFlowReversal),
              Line(
                points={{55,-85},{-60,-85}},
                color={0,128,255},
                visible=showDesignFlowDirection),
              Text(
                extent={{-149,-114},{151,-154}},
                lineColor={0,0,255},
                textString="%name"),
              Ellipse(
                extent={{-110,26},{-90,-24}},
                fillPattern=FillPattern.Solid,
                visible=port_a_exposesState),
              Ellipse(
                extent={{90,25},{110,-25}},
                fillPattern=FillPattern.Solid,
                visible=port_b_exposesState)}));
      end PartialTwoPort;

      connector HeatPorts_a
        "HeatPort connector with filled, large icon to be used for vectors of HeatPorts (vector dimensions must be added after dragging)"
        extends Modelica.Thermal.HeatTransfer.Interfaces.HeatPort;
        annotation (defaultComponentName="heatPorts_a",
             Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-200,-50},{200,50}},
              initialScale=0.2), graphics={
              Rectangle(
                extent={{-201,50},{200,-50}},
                lineColor={127,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-171,45},{-83,-45}},
                lineColor={127,0,0},
                fillColor={127,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-45,45},{43,-45}},
                lineColor={127,0,0},
                fillColor={127,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{82,45},{170,-45}},
                lineColor={127,0,0},
                fillColor={127,0,0},
                fillPattern=FillPattern.Solid)}));
      end HeatPorts_a;

      partial model PartialHeatTransfer "Common interface for heat transfer models"

        // Parameters
        replaceable package Medium=Modelica.Media.Interfaces.PartialMedium
          "Medium in the component"
          annotation(Dialog(tab="Internal Interface",enable=false));

        parameter Integer n=1 "Number of heat transfer segments"
          annotation(Dialog(tab="Internal Interface",enable=false), Evaluate=true);

        // Inputs provided to heat transfer model
        input Medium.ThermodynamicState[n] states
          "Thermodynamic states of flow segments";

        input SI.Area[n] surfaceAreas "Heat transfer areas";

        // Outputs defined by heat transfer model
        output SI.HeatFlowRate[n] Q_flows "Heat flow rates";

        // Parameters
        parameter Boolean use_k = false
          "= true to use k value for thermal isolation"
          annotation(Dialog(tab="Internal Interface",enable=false));
        parameter SI.CoefficientOfHeatTransfer k = 0
          "Heat transfer coefficient to ambient"
          annotation(Dialog(group="Ambient"),Evaluate=true);
        parameter SI.Temperature T_ambient = system.T_ambient "Ambient temperature"
          annotation(Dialog(group="Ambient"));
        outer Modelica.Fluid.System system "System wide properties";

        // Heat ports
        Modelica.Fluid.Interfaces.HeatPorts_a[n] heatPorts
          "Heat port to component boundary"
          annotation (Placement(transformation(extent={{-10,60},{10,80}}), iconTransformation(extent={{-20,60},{20,80}})));

        // Variables
        SI.Temperature[n] Ts = Medium.temperature(states)
          "Temperatures defined by fluid states";

      equation
        if use_k then
          Q_flows = heatPorts.Q_flow + {k*surfaceAreas[i]*(T_ambient - heatPorts[i].T) for i in 1:n};
        else
          Q_flows = heatPorts.Q_flow;
        end if;

        annotation (Documentation(info="<html>
<p>
This component is a common interface for heat transfer models. The heat flow rates <code>Q_flows[n]</code> through the boundaries of n flow segments
are obtained as function of the thermodynamic <code>states</code> of the flow segments for a given fluid <code>Medium</code>,
the <code>surfaceAreas[n]</code> and the boundary temperatures <code>heatPorts[n].T</code>.
</p>
<p>
The heat loss coefficient <code>k</code> can be used to model a thermal isolation between <code>heatPorts.T</code> and <code>T_ambient</code>.</p>
<p>
An extending model implementing this interface needs to define one equation: the relation between the predefined fluid temperatures <code>Ts[n]</code>,
the boundary temperatures <code>heatPorts[n].T</code>, and the heat flow rates <code>Q_flows[n]</code>.
</p>
</html>"));
      end PartialHeatTransfer;

        partial model PartialLumpedVolume
        "Lumped volume with mass and energy balance"
        import Modelica.Fluid.Types;
        import Modelica.Fluid.Types.Dynamics;
        import Modelica.Media.Interfaces.Choices.IndependentVariables;

          outer Modelica.Fluid.System system "System properties";
          replaceable package Medium =
            Modelica.Media.Interfaces.PartialMedium "Medium in the component"
              annotation (choicesAllMatching = true);

          // Inputs provided to the volume model
          input SI.Volume fluidVolume "Volume";

          // Assumptions
          parameter Types.Dynamics energyDynamics=system.energyDynamics
          "Formulation of energy balance"
            annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
          parameter Types.Dynamics massDynamics=system.massDynamics
          "Formulation of mass balance"
            annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
          final parameter Types.Dynamics substanceDynamics=massDynamics
          "Formulation of substance balance"
            annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));
          final parameter Types.Dynamics traceDynamics=massDynamics
          "Formulation of trace substance balance"
            annotation(Evaluate=true, Dialog(tab = "Assumptions", group="Dynamics"));

          // Initialization
          parameter Medium.AbsolutePressure p_start = system.p_start
          "Start value of pressure"
            annotation(Dialog(tab = "Initialization"));
          parameter Boolean use_T_start = true
          "= true, use T_start, otherwise h_start"
            annotation(Dialog(tab = "Initialization"), Evaluate=true);
          parameter Medium.Temperature T_start=
            if use_T_start then system.T_start else Medium.temperature_phX(p_start,h_start,X_start)
          "Start value of temperature"
            annotation(Dialog(tab = "Initialization", enable = use_T_start));
          parameter Medium.SpecificEnthalpy h_start=
            if use_T_start then Medium.specificEnthalpy_pTX(p_start, T_start, X_start) else Medium.h_default
          "Start value of specific enthalpy"
            annotation(Dialog(tab = "Initialization", enable = not use_T_start));
          parameter Medium.MassFraction X_start[Medium.nX] = Medium.X_default
          "Start value of mass fractions m_i/m"
            annotation (Dialog(tab="Initialization", enable=Medium.nXi > 0));
          parameter Medium.ExtraProperty C_start[Medium.nC](
               quantity=Medium.extraPropertiesNames) = Medium.C_default
          "Start value of trace substances"
            annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));

          Medium.BaseProperties medium(
            preferredMediumStates=true,
            p(start=p_start),
            h(start=h_start),
            T(start=T_start),
            Xi(start=X_start[1:Medium.nXi]));
          SI.Energy U "Internal energy of fluid";
          SI.Mass m "Mass of fluid";
          SI.Mass[Medium.nXi] mXi "Masses of independent components in the fluid";
          SI.Mass[Medium.nC] mC "Masses of trace substances in the fluid";
          // C need to be added here because unlike for Xi, which has medium.Xi,
          // there is no variable medium.C
          Medium.ExtraProperty C[Medium.nC] "Trace substance mixture content";

          // variables that need to be defined by an extending class
          SI.MassFlowRate mb_flow "Mass flows across boundaries";
          SI.MassFlowRate[Medium.nXi] mbXi_flow
          "Substance mass flows across boundaries";
          Medium.ExtraPropertyFlowRate[Medium.nC] mbC_flow
          "Trace substance mass flows across boundaries";
          SI.EnthalpyFlowRate Hb_flow
          "Enthalpy flow across boundaries or energy source/sink";
          SI.HeatFlowRate Qb_flow
          "Heat flow across boundaries or energy source/sink";
          SI.Power Wb_flow "Work flow across boundaries or source term";
      protected
          parameter Boolean initialize_p = not Medium.singleState
          "= true to set up initial equations for pressure";
          Real[Medium.nC] mC_scaled(min=fill(Modelica.Constants.eps, Medium.nC))
          "Scaled masses of trace substances in the fluid";
        equation
          assert(not (energyDynamics<>Dynamics.SteadyState and massDynamics==Dynamics.SteadyState) or Medium.singleState,
                 "Bad combination of dynamics options and Medium not conserving mass if fluidVolume is fixed.");

          // Total quantities
          m = fluidVolume*medium.d;
          mXi = m*medium.Xi;
          U = m*medium.u;
          mC = m*C;

          // Energy and mass balances
          if energyDynamics == Dynamics.SteadyState then
            0 = Hb_flow + Qb_flow + Wb_flow;
          else
            der(U) = Hb_flow + Qb_flow + Wb_flow;
          end if;

          if massDynamics == Dynamics.SteadyState then
            0 = mb_flow;
          else
            der(m) = mb_flow;
          end if;

          if substanceDynamics == Dynamics.SteadyState then
            zeros(Medium.nXi) = mbXi_flow;
          else
            der(mXi) = mbXi_flow;
          end if;

          if traceDynamics == Dynamics.SteadyState then
            zeros(Medium.nC)  = mbC_flow;
          else
            der(mC_scaled) = mbC_flow./Medium.C_nominal;
          end if;
            mC = mC_scaled.*Medium.C_nominal;

        initial equation
          // initialization of balances
          if energyDynamics == Dynamics.FixedInitial then
            /*
    if use_T_start then
      medium.T = T_start;
    else
      medium.h = h_start;
    end if;
    */
            if Medium.ThermoStates == IndependentVariables.ph or
               Medium.ThermoStates == IndependentVariables.phX then
               medium.h = h_start;
            else
               medium.T = T_start;
            end if;
          elseif energyDynamics == Dynamics.SteadyStateInitial then
            /*
    if use_T_start then
      der(medium.T) = 0;
    else
      der(medium.h) = 0;
    end if;
    */
            if Medium.ThermoStates == IndependentVariables.ph or
               Medium.ThermoStates == IndependentVariables.phX then
               der(medium.h) = 0;
            else
               der(medium.T) = 0;
            end if;
          end if;

          if massDynamics == Dynamics.FixedInitial then
            if initialize_p then
              medium.p = p_start;
            end if;
          elseif massDynamics == Dynamics.SteadyStateInitial then
            if initialize_p then
              der(medium.p) = 0;
            end if;
          end if;

          if substanceDynamics == Dynamics.FixedInitial then
            medium.Xi = X_start[1:Medium.nXi];
          elseif substanceDynamics == Dynamics.SteadyStateInitial then
            der(medium.Xi) = zeros(Medium.nXi);
          end if;

          if traceDynamics == Dynamics.FixedInitial then
            mC_scaled = m*C_start[1:Medium.nC]./Medium.C_nominal;
          elseif traceDynamics == Dynamics.SteadyStateInitial then
            der(mC_scaled) = zeros(Medium.nC);
          end if;

          annotation (
            Documentation(info="<html>
<p>
Interface and base class for an ideally mixed fluid volume with the ability to store mass and energy.
The following boundary flow and source terms are part of the energy balance and must be specified in an extending class:
</p>
<ul>
<li><code><strong>Qb_flow</strong></code>, e.g., convective or latent heat flow rate across segment boundary, and</li>
<li><code><strong>Wb_flow</strong></code>, work term, e.g., p*der(fluidVolume) if the volume is not constant.</li>
</ul>
<p>
The component volume <code><strong>fluidVolume</strong></code> is an input that needs to be set in the extending class to complete the model.
</p>
<p>
Further source terms must be defined by an extending class for fluid flow across the segment boundary:
</p>
<ul>
<li><code><strong>Hb_flow</strong></code>, enthalpy flow,</li>
<li><code><strong>mb_flow</strong></code>, mass flow,</li>
<li><code><strong>mbXi_flow</strong></code>, substance mass flow, and</li>
<li><code><strong>mbC_flow</strong></code>, trace substance mass flow.</li>
</ul>
</html>"));
        end PartialLumpedVolume;
      annotation (Documentation(info="<html>

</html>",     revisions="<html>
<ul>
<li><em>June 9th, 2008</em>
       by Michael Sielemann: Introduced stream keyword after decision at 57th Design Meeting (Lund).</li>
<li><em>May 30, 2007</em>
       by Christoph Richter: moved everything back to its original position in Modelica.Fluid.</li>
<li><em>Apr. 20, 2007</em>
       by Christoph Richter: moved parts of the original package from Modelica.Fluid
       to the development branch of Modelica 2.2.2.</li>
<li><em>Nov. 2, 2005</em>
       by Francesco Casella: restructured after 45th Design Meeting.</li>
<li><em>Nov. 20-21, 2002</em>
       by Hilding Elmqvist, Mike Tiller, Allan Watson, John Batteh, Chuck Newman,
       Jonas Eborn: Improved at the 32nd Modelica Design Meeting.
<li><em>Nov. 11, 2002</em>
       by Hilding Elmqvist, Martin Otter: improved version.</li>
<li><em>Nov. 6, 2002</em>
       by Hilding Elmqvist: first version.</li>
<li><em>Aug. 11, 2002</em>
       by Martin Otter: Improved according to discussion with Hilding
       Elmqvist and Hubertus Tummescheit.<br>
       The PortVicinity model is manually
       expanded in the base models.<br>
       The Volume used for components is renamed
       PartialComponentVolume.<br>
       A new volume model \"Fluid.Components.PortVolume\"
       introduced that has the medium properties of the port to which it is
       connected.<br>
       Fluid.Interfaces.PartialTwoPortTransport is a component
       for elementary two port transport elements, whereas PartialTwoPort
       is a component for a container component.</li>
</ul>
</html>"));
    end Interfaces;

    package Types "Common types for fluid models"
      extends Modelica.Icons.TypesPackage;

      type Dynamics = enumeration(
          DynamicFreeInitial
            "DynamicFreeInitial -- Dynamic balance, Initial guess value",
          FixedInitial "FixedInitial -- Dynamic balance, Initial value fixed",
          SteadyStateInitial
            "SteadyStateInitial -- Dynamic balance, Steady state initial with guess value",
          SteadyState "SteadyState -- Steady state balance, Initial guess value")
        "Enumeration to define definition of balance equations"
      annotation (Documentation(info="<html>
<p>
Enumeration to define the formulation of balance equations
(to be selected via choices menu):
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><strong>Dynamics.</strong></th><th><strong>Meaning</strong></th></tr>
<tr><td>DynamicFreeInitial</td><td>Dynamic balance, Initial guess value</td></tr>

<tr><td>FixedInitial</td><td>Dynamic balance, Initial value fixed</td></tr>

<tr><td>SteadyStateInitial</td><td>Dynamic balance, Steady state initial with guess value</td></tr>

<tr><td>SteadyState</td><td>Steady state balance, Initial guess value</td></tr>
</table>

<p>
The enumeration \"Dynamics\" is used for the mass, energy and momentum balance equations
respectively. The exact meaning for the three balance equations is stated in the following
tables:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><td colspan=\"3\"><strong>Mass balance</strong> </td></tr>
<tr><td><strong>Dynamics.</strong></td>
    <td><strong>Balance equation</strong></td>
    <td><strong>Initial condition</strong></td></tr>

<tr><td> DynamicFreeInitial</td>
    <td> no restrictions </td>
    <td> no initial conditions </td></tr>

<tr><td> FixedInitial</td>
    <td> no restrictions </td>
    <td> <strong>if</strong> Medium.singleState <strong>then</strong><br>
         &nbsp;&nbsp;no initial condition<br>
         <strong>else</strong> p=p_start </td></tr>

<tr><td> SteadyStateInitial</td>
    <td> no restrictions </td>
    <td> <strong>if</strong> Medium.singleState <strong>then</strong><br>
         &nbsp;&nbsp;no initial condition<br>
         <strong>else</strong> <strong>der</strong>(p)=0 </td></tr>

<tr><td> SteadyState</td>
    <td> <strong>der</strong>(m)=0  </td>
    <td> no initial conditions </td></tr>
</table>

&nbsp;<br>

<table border=1 cellspacing=0 cellpadding=2>
<tr><td colspan=\"3\"><strong>Energy balance</strong> </td></tr>
<tr><td><strong>Dynamics.</strong></td>
    <td><strong>Balance equation</strong></td>
    <td><strong>Initial condition</strong></td></tr>

<tr><td> DynamicFreeInitial</td>
    <td> no restrictions </td>
    <td> no initial conditions </td></tr>

<tr><td> FixedInitial</td>
    <td> no restrictions </td>
    <td> T=T_start or h=h_start </td></tr>

<tr><td> SteadyStateInitial</td>
    <td> no restrictions </td>
    <td> <strong>der</strong>(T)=0 or <strong>der</strong>(h)=0 </td></tr>

<tr><td> SteadyState</td>
    <td> <strong>der</strong>(U)=0  </td>
    <td> no initial conditions </td></tr>
</table>

&nbsp;<br>

<table border=1 cellspacing=0 cellpadding=2>
<tr><td colspan=\"3\"><strong>Momentum balance</strong> </td></tr>
<tr><td><strong>Dynamics.</strong></td>
    <td><strong>Balance equation</strong></td>
    <td><strong>Initial condition</strong></td></tr>

<tr><td> DynamicFreeInitial</td>
    <td> no restrictions </td>
    <td> no initial conditions </td></tr>

<tr><td> FixedInitial</td>
    <td> no restrictions </td>
    <td> m_flow = m_flow_start </td></tr>

<tr><td> SteadyStateInitial</td>
    <td> no restrictions </td>
    <td> <strong>der</strong>(m_flow)=0 </td></tr>

<tr><td> SteadyState</td>
    <td> <strong>der</strong>(m_flow)=0 </td>
    <td> no initial conditions </td></tr>
</table>

<p>
In the tables above, the equations are given for one-substance fluids. For multiple-substance
fluids and for trace substances, equivalent equations hold.
</p>

<p>
Medium.singleState is a medium property and defines whether the medium is only
described by one state (+ the mass fractions in case of a multi-substance fluid). In such
a case one initial condition less must be provided. For example, incompressible
media have Medium.singleState = <strong>true</strong>.
</p>

</html>"));

      type PortFlowDirection = enumeration(
          Entering "Fluid flow is only entering",
          Leaving "Fluid flow is only leaving",
          Bidirectional "No restrictions on fluid flow (flow reversal possible)")
        "Enumeration to define whether flow reversal is allowed" annotation (
          Documentation(info="<html>

<p>
Enumeration to define the assumptions on the model for the
direction of fluid flow at a port (to be selected via choices menu):
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><strong>PortFlowDirection.</strong></th>
    <th><strong>Meaning</strong></th></tr>

<tr><td>Entering</td>
    <td>Fluid flow is only entering the port from the outside</td></tr>

<tr><td>Leaving</td>
    <td>Fluid flow is only leaving the port to the outside</td></tr>

<tr><td>Bidirectional</td>
    <td>No restrictions on fluid flow (flow reversal possible)</td></tr>
</table>

<p>
The default is \"PortFlowDirection.Bidirectional\". If you are completely sure that
the flow is only in one direction, then the other settings may
make the simulation of your model faster.
</p>

</html>"));
      annotation (preferredView="info",
                  Documentation(info="<html>

</html>"));
    end Types;

    package Utilities "Utility models to construct fluid components (should not be used directly)"
      extends Modelica.Icons.UtilitiesPackage;

      function checkBoundary "Check whether boundary definition is correct"
        extends Modelica.Icons.Function;
        input String mediumName;
        input String substanceNames[:] "Names of substances";
        input Boolean singleState;
        input Boolean define_p;
        input Real X_boundary[:];
        input String modelName = "??? boundary ???";
      protected
        Integer nX = size(X_boundary,1);
        String X_str;
      algorithm
        assert(not singleState or singleState and define_p, "
Wrong value of parameter define_p (= false) in model \""     + modelName + "\":
The selected medium \""     + mediumName + "\" has Medium.singleState=true.
Therefore, an boundary density cannot be defined and
define_p = true is required.
");

        for i in 1:nX loop
          assert(X_boundary[i] >= 0.0, "
Wrong boundary mass fractions in medium \""
      + mediumName + "\" in model \"" + modelName + "\":
The boundary value X_boundary("   + String(i) + ") = " + String(
            X_boundary[i]) + "
is negative. It must be positive.
");     end for;

        if nX > 0 and abs(sum(X_boundary) - 1.0) > 1e-10 then
           X_str :="";
           for i in 1:nX loop
              X_str :=X_str + "   X_boundary[" + String(i) + "] = " + String(X_boundary[
              i]) + " \"" + substanceNames[i] + "\"\n";
           end for;
           Modelica.Utilities.Streams.error(
              "The boundary mass fractions in medium \"" + mediumName + "\" in model \"" + modelName + "\"\n" +
              "do not sum up to 1. Instead, sum(X_boundary) = " + String(sum(X_boundary)) + ":\n"
              + X_str);
        end if;
      end checkBoundary;

      function regSquare2
        "Anti-symmetric approximation of square with discontinuous factor so that the first derivative is non-zero and is continuous"
        extends Modelica.Icons.Function;
        input Real x "abscissa value";
        input Real x_small(min=0)=0.01
          "approximation of function for |x| <= x_small";
        input Real k1(min=0)=1 "y = (if x>=0 then k1 else k2)*x*|x|";
        input Real k2(min=0)=1 "y = (if x>=0 then k1 else k2)*x*|x|";
        input Boolean use_yd0 = false "= true, if yd0 shall be used";
        input Real yd0(min=0)=1 "Desired derivative at x=0: dy/dx = yd0";
        output Real y "ordinate value";
      protected
        encapsulated function regSquare2_utility
          "Interpolating with two 3-order polynomials with a prescribed derivative at x=0"
          import Modelica;
          extends Modelica.Icons.Function;
          import Modelica.Fluid.Utilities.evaluatePoly3_derivativeAtZero;
           input Real x;
           input Real x1 "approximation of function abs(x) < x1";
           input Real k1 "y = (if x>=0 then k1 else -k2)*x*|x|; k1 >= k2";
           input Real k2 "y = (if x>=0 then k1 else -k2)*x*|x|";
           input Boolean use_yd0 = false "= true, if yd0 shall be used";
           input Real yd0(min=0)=1 "Desired derivative at x=0: dy/dx = yd0";
           output Real y;
        protected
           Real x2;
           Real y1;
           Real y2;
           Real y1d;
           Real y2d;
           Real w;
           Real w1;
           Real w2;
           Real y0d;
           Real ww;
        algorithm
           // x2 :=-x1*(k2/k1)^2;
           x2 := -x1;
           if x <= x2 then
              y := -k2*x^2;
           else
               y1 := k1*x1^2;
               y2 :=-k2*x2^2;
              y1d := k1*2*x1;
              y2d :=-k2*2*x2;
              if use_yd0 then
                 y0d :=yd0;
              else
                 /* Determine derivative, such that first and second derivative
              of left and right polynomial are identical at x=0:
              see derivation in function regRoot2
           */
                 w :=x2/x1;
                 y0d := ( (3*y2 - x2*y2d)/w - (3*y1 - x1*y1d)*w) /(2*x1*(1 - w));
              end if;

              /* Modify derivative y0d, such that the polynomial is
           monotonically increasing. A sufficient condition is
             0 <= y0d <= sqrt(5)*k_i*|x_i|
        */
              w1 :=sqrt(5)*k1*x1;
              w2 :=sqrt(5)*k2*abs(x2);
              // y0d :=min(y0d, 0.9*min(w1, w2));
              ww :=0.9*(if w1 < w2 then w1 else w2);
              if ww < y0d then
                 y0d :=ww;
              end if;
              y := if x >= 0 then evaluatePoly3_derivativeAtZero(x,x1,y1,y1d,y0d) else
                                  evaluatePoly3_derivativeAtZero(x,x2,y2,y2d,y0d);
           end if;
           annotation(smoothOrder=2);
        end regSquare2_utility;
      algorithm
        y := smooth(2,if x >= x_small then k1*x^2 else
                      if x <= -x_small then -k2*x^2 else
                      if k1 >= k2 then regSquare2_utility(x,x_small,k1,k2,use_yd0,yd0) else
                                      -regSquare2_utility(-x,x_small,k2,k1,use_yd0,yd0));
        annotation(smoothOrder=2, Documentation(info="<html>
<p>
Approximates the function
</p>
<pre>
   y = <strong>if</strong> x &ge; 0 <strong>then</strong> k1*x*x <strong>else</strong> -k2*x*x, with k1, k2 > 0
</pre>
<p>
in such a way that within the region -x_small &le; x &le; x_small,
the function is described by two polynomials of third order
(one in the region -x_small .. 0 and one within the region 0 .. x_small)
such that
</p>

<ul>
<li> The derivative at x=0 is non-zero (in order that the
     inverse of the function does not have an infinite derivative).</li>
<li> The overall function is continuous with a
     continuous first derivative everywhere.</li>
<li> If parameter use_yd0 = <strong>false</strong>, the two polynomials
     are constructed such that the second derivatives at x=0
     are identical. If use_yd0 = <strong>true</strong>, the derivative
     at x=0 is explicitly provided via the additional argument
     yd0. If necessary, the derivative yd0 is automatically
     reduced in order that the polynomials are strict monotonically
     increasing <em>[Fritsch and Carlson, 1980]</em>.</li>
</ul>

<p>
A typical screenshot for k1=1, k2=3 is shown in the next figure:
</p>
<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/regSquare2_b.png\"
     alt=\"regSquare2_b.png\">
</p>

<p>
The (smooth, non-zero) derivative of the function with
k1=1, k2=3 is shown in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/regSquare2_c.png\"
     alt=\"regSquare2_b.png\">
</p>

<p>
<strong>Literature</strong>
</p>

<dl>
<dt> Fritsch F.N. and Carlson R.E. (1980):</dt>
<dd> <strong>Monotone piecewise cubic interpolation</strong>.
     SIAM J. Numerc. Anal., Vol. 17, No. 2, April 1980, pp. 238-246</dd>
</dl>
</html>",     revisions="<html>
<ul>
<li><em>Nov., 2005</em>
    by <a href=\"mailto:Martin.Otter@DLR.de\">Martin Otter</a>:<br>
    Designed and implemented.</li>
</ul>
</html>"));
      end regSquare2;

      function regStep
        "Approximation of a general step, such that the characteristic is continuous and differentiable"
        extends Modelica.Icons.Function;
        input Real x "Abscissa value";
        input Real y1 "Ordinate value for x > 0";
        input Real y2 "Ordinate value for x < 0";
        input Real x_small(min=0) = 1e-5
          "Approximation of step for -x_small <= x <= x_small; x_small >= 0 required";
        output Real y "Ordinate value to approximate y = if x > 0 then y1 else y2";
      algorithm
        y := smooth(1, if x >  x_small then y1 else
                       if x < -x_small then y2 else
                       if x_small > 0 then (x/x_small)*((x/x_small)^2 - 3)*(y2-y1)/4 + (y1+y2)/2 else (y1+y2)/2);
        annotation(Documentation(revisions="<html>
<ul>
<li><em>April 29, 2008</em>
    by <a href=\"mailto:Martin.Otter@DLR.de\">Martin Otter</a>:<br>
    Designed and implemented.</li>
<li><em>August 12, 2008</em>
    by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br>
    Minor modification to cover the limit case <code>x_small -> 0</code> without division by zero.</li>
</ul>
</html>",       info="<html>
<p>
This function is used to approximate the equation
</p>
<pre>
    y = <strong>if</strong> x &gt; 0 <strong>then</strong> y1 <strong>else</strong> y2;
</pre>

<p>
by a smooth characteristic, so that the expression is continuous and differentiable:
</p>

<pre>
   y = <strong>smooth</strong>(1, <strong>if</strong> x &gt;  x_small <strong>then</strong> y1 <strong>else</strong>
                 <strong>if</strong> x &lt; -x_small <strong>then</strong> y2 <strong>else</strong> f(y1, y2));
</pre>

<p>
In the region -x_small &lt; x &lt; x_small a 2nd order polynomial is used
for a smooth transition from y1 to y2.
</p>
</html>"));
      end regStep;

      function evaluatePoly3_derivativeAtZero
        "Evaluate polynomial of order 3 that passes the origin with a predefined derivative"
        extends Modelica.Icons.Function;
        input Real x "Value for which polynomial shall be evaluated";
        input Real x1 "Abscissa value";
        input Real y1 "y1=f(x1)";
        input Real y1d "First derivative at y1";
        input Real y0d "First derivative at f(x=0)";
        output Real y;
      protected
        Real a1;
        Real a2;
        Real a3;
        Real xx;
      algorithm
        a1 := x1*y0d;
        a2 := 3*y1 - x1*y1d - 2*a1;
        a3 := y1 - a2 - a1;
        xx := x/x1;
        y  := xx*(a1 + xx*(a2 + xx*a3));
        annotation(smoothOrder=3, Documentation(info="<html>

</html>"));
      end evaluatePoly3_derivativeAtZero;

      function regFun3 "Co-monotonic and C1 smooth regularization function"
        extends Modelica.Icons.Function;

        input Real x "Abscissa value";
        input Real x0 "Lower abscissa value";
        input Real x1 "Upper abscissa value";
        input Real y0 "Ordinate value at lower abscissa value";
        input Real y1 "Ordinate value at upper abscissa value";
        input Real y0d "Derivative at lower abscissa value";
        input Real y1d "Derivative at upper abscissa value";

        output Real y "Ordinate value";
        output Real c
          "Slope of linear section between two cubic polynomials or dummy linear section slope if single cubic is used";

      protected
        Real h0 "Width of interval i=0";
        Real Delta0 "Slope of secant on interval i=0";
        Real xstar "Inflection point of cubic polynomial S0";
        Real mu "Distance of inflection point and left limit x0";
        Real eta "Distance of right limit x1 and inflection point";
        Real omega "Slope of cubic polynomial S0 at inflection point";
        Real rho "Weighting factor of eta and eta_tilde, mu and mu_tilde";
        Real theta0 "Slope metric";
        Real mu_tilde "Distance of start of linear section and left limit x0";
        Real eta_tilde "Distance of right limit x1 and end of linear section";
        Real xi1 "Start of linear section";
        Real xi2 "End of linear section";
        Real a1 "Leading coefficient of cubic on the left";
        Real a2 "Leading coefficient of cubic on the right";
        Real const12 "Integration constant of left cubic, linear section";
        Real const3 "Integration constant of right cubic";
        Real aux01;
        Real aux02;
        Boolean useSingleCubicPolynomial=false
          "Indicate to override further logic and use single cubic";
      algorithm
        // Check arguments: Data point position
        assert(x0 < x1, "regFun3(): Data points not sorted appropriately (x0 = " +
          String(x0) + " > x1 = " + String(x1) + "). Please flip arguments.");
        // Check arguments: Data point derivatives
        if y0d*y1d >= 0 then
          // Derivatives at data points allow co-monotone interpolation, nothing to do
        else
          // Strictly speaking, derivatives at data points do not allow co-monotone interpolation, however, they may be numerically zero so assert this
          assert(abs(y0d)<Modelica.Constants.eps or abs(y1d)<Modelica.Constants.eps, "regFun3(): Derivatives at data points do not allow co-monotone interpolation, as both are non-zero, of opposite sign and have an absolute value larger than machine eps (y0d = " +
          String(y0d) + ", y1d = " + String(y1d) + "). Please correct arguments.");
        end if;

        h0 := x1 - x0;
        Delta0 := (y1 - y0)/h0;

        if abs(Delta0) <= 0 then
          // Points (x0,y0) and (x1,y1) on horizontal line
          // Degenerate case as we cannot fulfill the C1 goal an comonotone behaviour at the same time
          y := y0 + Delta0*(x-x0);     // y == y0 == y1 with additional term to assist automatic differentiation
          c := 0;
        elseif abs(y1d + y0d - 2*Delta0) < 100*Modelica.Constants.eps then
          // Inflection point at +/- infinity, thus S0 is co-monotone and can be returned directly
          y := y0 + (x-x0)*(y0d + (x-x0)/h0*( (-2*y0d-y1d+3*Delta0) + (x-x0)*(y0d+y1d-2*Delta0)/h0));
          // Provide a "dummy linear section slope" as the slope of the cubic at x:=(x0+x1)/2
          aux01 := (x0 + x1)/2;
          c := 3*(y0d + y1d - 2*Delta0)*(aux01 - x0)^2/h0^2 + 2*(-2*y0d - y1d + 3*Delta0)*(aux01 - x0)/h0
             + y0d;
        else
          // Points (x0,y0) and (x1,y1) not on horizontal line and inflection point of S0 not at +/- infinity
          // Do actual interpolation
          xstar := 1/3*(-3*x0*y0d - 3*x0*y1d + 6*x0*Delta0 - 2*h0*y0d - h0*y1d + 3*h0*
            Delta0)/(-y0d - y1d + 2*Delta0);
          mu := xstar - x0;
          eta := x1 - xstar;
          omega := 3*(y0d + y1d - 2*Delta0)*(xstar - x0)^2/h0^2 + 2*(-2*y0d - y1d + 3*
            Delta0)*(xstar - x0)/h0 + y0d;

          aux01 := 0.25*sign(Delta0)*min(abs(omega), abs(Delta0))
            "Slope c if not using plain cubic S0";
          if abs(y0d - y1d) <= 100*Modelica.Constants.eps then
            // y0 == y1 (value and sign equal) -> resolve indefinite 0/0
            aux02 := y0d;
            if y1 > y0 + y0d*(x1 - x0) then
              // If y1 is above the linear extension through (x0/y0)
              // with slope y0d (when slopes are identical)
              //  -> then always used single cubic polynomial
              useSingleCubicPolynomial := true;
            end if;
          elseif abs(y1d + y0d - 2*Delta0) < 100*Modelica.Constants.eps then
            // (y1d+y0d-2*Delta0) approximately 0 -> avoid division by 0
            aux02 := (6*Delta0*(y1d + y0d - 3/2*Delta0) - y1d*y0d - y1d^2 - y0d^2)*(
              if (y1d + y0d - 2*Delta0) >= 0 then 1 else -1)*Modelica.Constants.inf;
          else
            // Okay, no guarding necessary
            aux02 := (6*Delta0*(y1d + y0d - 3/2*Delta0) - y1d*y0d - y1d^2 - y0d^2)/(3*
              (y1d + y0d - 2*Delta0));
          end if;

          //aux02 := -1/3*(y0d^2+y0d*y1d-6*y0d*Delta0+y1d^2-6*y1d*Delta0+9*Delta0^2)/(y0d+y1d-2*Delta0);
          //aux02 := -1/3*(6*y1d*y0*x1+y0d*y1d*x1^2-6*y0d*x0*y0+y0d^2*x0^2+y0d^2*x1^2+y1d^2*x1^2+y1d^2*x0^2-2*y0d*x0*y1d*x1-2*x0*y0d^2*x1+y0d*y1d*x0^2+6*y0d*x0*y1-6*y0d*y1*x1+6*y0d*y0*x1-2*x0*y1d^2*x1-6*y1d*y1*x1+6*y1d*x0*y1-6*y1d*x0*y0-18*y1*y0+9*y1^2+9*y0^2)/(y0d*x1^2-2*x0*y0d*x1+y1d*x1^2-2*x0*y1d*x1-2*y1*x1+2*y0*x1+y0d*x0^2+y1d*x0^2+2*x0*y1-2*x0*y0);

          // Test criteria (also used to avoid saddle points that lead to integrator contraction):
          //
          //  1. Cubic is not monotonic (from Gasparo Morandi)
          //       ((mu > 0) and (eta < h0) and (Delta0*omega <= 0))
          //
          //  2. Cubic may be monotonic but the linear section slope c is either too close
          //     to zero or the end point of the linear section is left of the start point
          //     Note however, that the suggested slope has to have the same sign as Delta0.
          //       (abs(aux01)<abs(aux02) and aux02*Delta0>=0)
          //
          //  3. Cubic may be monotonic but the resulting slope in the linear section
          //     is too close to zero (less than 1/10 of Delta0).
          //       (c < Delta0 / 10)
          //
          if (((mu > 0) and (eta < h0) and (Delta0*omega <= 0)) or (abs(aux01) < abs(
              aux02) and aux02*Delta0 >= 0) or (abs(aux01) < abs(0.1*Delta0))) and
              not useSingleCubicPolynomial then
            // NOT monotonic using plain cubic S0, use piecewise function S0 tilde instead
            c := aux01;
            // Avoid saddle points that are co-monotonic but lead to integrator contraction
            if abs(c) < abs(aux02) and aux02*Delta0 >= 0 then
              c := aux02;
            end if;
            if abs(c) < abs(0.1*Delta0) then
              c := 0.1*Delta0;
            end if;
            theta0 := (y0d*mu + y1d*eta)/h0;
            if abs(theta0 - c) < 1e-6 then
              // Slightly reduce c in order to avoid ill-posed problem
              c := (1 - 1e-6)*theta0;
            end if;
            rho := 3*(Delta0 - c)/(theta0 - c);
            mu_tilde := rho*mu;
            eta_tilde := rho*eta;
            xi1 := x0 + mu_tilde;
            xi2 := x1 - eta_tilde;
            a1 := (y0d - c)/max(mu_tilde^2, 100*Modelica.Constants.eps);
            a2 := (y1d - c)/max(eta_tilde^2, 100*Modelica.Constants.eps);
            const12 := y0 - a1/3*(x0 - xi1)^3 - c*x0;
            const3 := y1 - a2/3*(x1 - xi2)^3 - c*x1;
            // Do actual interpolation
            if (x < xi1) then
              y := a1/3*(x - xi1)^3 + c*x + const12;
            elseif (x < xi2) then
              y := c*x + const12;
            else
              y := a2/3*(x - xi2)^3 + c*x + const3;
            end if;
          else
            // Cubic S0 is monotonic, use it as is
            y := y0 + (x-x0)*(y0d + (x-x0)/h0*( (-2*y0d-y1d+3*Delta0) + (x-x0)*(y0d+y1d-2*Delta0)/h0));
            // Provide a "dummy linear section slope" as the slope of the cubic at x:=(x0+x1)/2
            aux01 := (x0 + x1)/2;
            c := 3*(y0d + y1d - 2*Delta0)*(aux01 - x0)^2/h0^2 + 2*(-2*y0d - y1d + 3*Delta0)*(aux01 - x0)/h0
               + y0d;
          end if;
        end if;

        annotation (smoothOrder=1, Documentation(revisions="<html>
<ul>
<li><em>May 2008</em> by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br/>Designed and implemented.</li>
<li><em>February 2011</em> by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br/>If the inflection point of the cubic S0 was at +/- infinity, the test criteria of <em>[Gasparo and Morandi, 1991]</em> result in division by zero. This case is handled properly now.</li>
<li><em>March 2013</em> by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br/>If the arguments prescribed a degenerate case with points <code>(x0,y0)</code> and <code>(x1,y1)</code> on horizontal line, then return value <code>c</code> was undefined. This was corrected. Furthermore, an additional term was included for the computation of <code>y</code> in this case to assist automatic differentiation.</li>
</ul>
</html>",       info="<html>
<p>
Approximates a function in a region between <code>x0</code> and <code>x1</code>
such that
</p>
<ul>
<li> The overall function is continuous with a
     continuous first derivative everywhere.</li>
<li> The function is co-monotone with the given
     data points.</li>
</ul>
<p>
In this region, a continuation is constructed from the given points
<code>(x0, y0)</code>, <code>(x1, y1)</code> and the respective
derivatives. For this purpose, a single polynomial of third order or two
cubic polynomials with a linear section in between are used <em>[Gasparo
and Morandi, 1991]</em>. This algorithm was extended with two additional
conditions to avoid saddle points with zero/infinite derivative that lead to
integrator step size reduction to zero.
</p>
<p>
This function was developed for pressure loss correlations properly
addressing the static head on top of the established requirements
for monotonicity and smoothness. In this case, the present function
allows to implement the exact solution in the limit of
<code>x1-x0 -> 0</code> or <code>y1-y0 -> 0</code>.
</p>
<p>
Typical screenshots for two different configurations
are shown below. The first one illustrates five different settings of <code>xi</code> and <code>yid</code>:
</p>
<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/regFun3_a.png\"
      alt=\"regFun3_a.png\">
</p>
<p>
The second graph shows the continuous derivative of this regularization function:
</p>
<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/Components/regFun3_b.png\"
     alt=\"regFun3_a.png\">
</p>

<p>
<strong>Literature</strong>
</p>

<dl>
<dt> Gasparo M. G. and Morandi R. (1991):</dt>
<dd> <strong>Piecewise cubic monotone interpolation with assigned slopes</strong>.
     Computing, Vol. 46, Issue 4, December 1991, pp. 355 - 365.</dd>
</dl>
</html>"));
      end regFun3;

      function cubicHermite_withDerivative
        "Evaluate a cubic Hermite spline, return value and derivative"
        extends Modelica.Icons.Function;

        input Real x "Abscissa value";
        input Real x1 "Lower abscissa value";
        input Real x2 "Upper abscissa value";
        input Real y1 "Lower ordinate value";
        input Real y2 "Upper ordinate value";
        input Real y1d "Lower gradient";
        input Real y2d "Upper gradient";
        output Real y "Interpolated ordinate value";
        output Real dy_dx "Derivative dy/dx at abscissa value x";
      protected
        Real h "Distance between x1 and x2";
        Real t "abscissa scaled with h, i.e., t=[0..1] within x=[x1..x2]";
        Real h00 "Basis function 00 of cubic Hermite spline";
        Real h10 "Basis function 10 of cubic Hermite spline";
        Real h01 "Basis function 01 of cubic Hermite spline";
        Real h11 "Basis function 11 of cubic Hermite spline";

        Real h00d "d/dt h00";
        Real h10d "d/dt h10";
        Real h01d "d/dt h01";
        Real h11d "d/dt h11";

        Real aux3 "t cube";
        Real aux2 "t square";
      algorithm
        h := x2 - x1;
        if abs(h)>0 then
          // Regular case
          t := (x - x1)/h;

          aux3 :=t^3;
          aux2 :=t^2;

          h00 := 2*aux3 - 3*aux2 + 1;
          h10 := aux3 - 2*aux2 + t;
          h01 := -2*aux3 + 3*aux2;
          h11 := aux3 - aux2;

          h00d := 6*(aux2 - t);
          h10d := 3*aux2 - 4*t + 1;
          h01d := 6*(t - aux2);
          h11d := 3*aux2 - 2*t;

          y := y1*h00 + h*y1d*h10 + y2*h01 + h*y2d*h11;
          dy_dx := y1*h00d/h + y1d*h10d + y2*h01d/h + y2d*h11d;
        else
          // Degenerate case, x1 and x2 are identical, return step function
          y := (y1 + y2)/2;
          dy_dx := sign(y2 - y1)*Modelica.Constants.inf;
        end if;
        annotation(smoothOrder=3, Documentation(revisions="<html>
<ul>
<li><em>May 2008</em>
    by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br>
    Designed and implemented.</li>
</ul>
</html>"));
      end cubicHermite_withDerivative;
      annotation (Documentation(info="<html>

</html>"));
    end Utilities;
  annotation (Icon(graphics={
          Polygon(points={{-70,26},{68,-44},{68,26},{2,-10},{-70,-42},{-70,26}}),
          Line(points={{2,42},{2,-10}}),
          Rectangle(
            extent={{-18,50},{22,42}},
            fillPattern=FillPattern.Solid)}), preferredView="info",
    Documentation(info="<html>
<p>
Library <strong>Modelica.Fluid</strong> is a <strong>free</strong> Modelica package providing components for
<strong>1-dimensional thermo-fluid flow</strong> in networks of vessels, pipes, fluid machines, valves and fittings.
A unique feature is that the component equations and the media models
as well as pressure loss and heat transfer correlations are decoupled from each other.
All components are implemented such that they can be used for
media from the Modelica.Media library. This means especially that an
incompressible or compressible medium, a single or a multiple
substance medium with one or more phases might be used.
</p>

<p>
In the next figure, several features of the library are demonstrated with
a simple heating system with a closed flow cycle. By just changing one configuration parameter in the system object the equations are changed between steady-state and dynamic simulation with fixed or steady-state initial conditions.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Fluid/UsersGuide/HeatingSystem.png\" border=\"1\"
     alt=\"HeatingSystem.png\">
</p>

<p>
With respect to previous versions, the design
of the connectors has been changed in a non-backward compatible way,
using the recently developed concept
of stream connectors that results in much more reliable simulations
(see also <a href=\"modelica://Modelica/Resources/Documentation/Fluid/Stream-Connectors-Overview-Rationale.pdf\">Stream-Connectors-Overview-Rationale.pdf</a>).
This extension was included in Modelica 3.1.
</p>

<p>
The following parts are useful, when newly starting with this library:
</p>
<ul>
<li> <a href=\"modelica://Modelica.Fluid.UsersGuide\">Modelica.Fluid.UsersGuide</a>.</li>
<li> <a href=\"modelica://Modelica.Fluid.UsersGuide.ReleaseNotes\">Modelica.Fluid.UsersGuide.ReleaseNotes</a>
     summarizes the changes of the library releases.</li>
<li> <a href=\"modelica://Modelica.Fluid.Examples\">Modelica.Fluid.Examples</a>
     contains examples that demonstrate the usage of this library.</li>
</ul>
<p>
Copyright &copy; 2002-2019, Modelica Association and contributors
</p>
</html>"));
  end Fluid;

  package Media "Library of media property models"
  extends Modelica.Icons.Package;
  import SI = Modelica.SIunits;
  import Cv = Modelica.SIunits.Conversions;

  package Interfaces "Interfaces for media models"
    extends Modelica.Icons.InterfacesPackage;

    partial package PartialMedium
      "Partial medium properties (base package of all media packages)"
      extends Modelica.Media.Interfaces.Types;
      extends Modelica.Icons.MaterialPropertiesPackage;

      // Constants to be set in Medium
      constant Modelica.Media.Interfaces.Choices.IndependentVariables
        ThermoStates "Enumeration type for independent variables";
      constant String mediumName="unusablePartialMedium" "Name of the medium";
      constant String substanceNames[:]={mediumName}
        "Names of the mixture substances. Set substanceNames={mediumName} if only one substance.";
      constant String extraPropertiesNames[:]=fill("", 0)
        "Names of the additional (extra) transported properties. Set extraPropertiesNames=fill(\"\",0) if unused";
      constant Boolean singleState
        "= true, if u and d are not a function of pressure";
      constant Boolean reducedX=true
        "= true if medium contains the equation sum(X) = 1.0; set reducedX=true if only one substance (see docu for details)";
      constant Boolean fixedX=false
        "= true if medium contains the equation X = reference_X";
      constant AbsolutePressure reference_p=101325
        "Reference pressure of Medium: default 1 atmosphere";
      constant Temperature reference_T=298.15
        "Reference temperature of Medium: default 25 deg Celsius";
      constant MassFraction reference_X[nX]=fill(1/nX, nX)
        "Default mass fractions of medium";
      constant AbsolutePressure p_default=101325
        "Default value for pressure of medium (for initialization)";
      constant Temperature T_default=Modelica.SIunits.Conversions.from_degC(20)
        "Default value for temperature of medium (for initialization)";
      constant SpecificEnthalpy h_default=specificEnthalpy_pTX(
              p_default,
              T_default,
              X_default)
        "Default value for specific enthalpy of medium (for initialization)";
      constant MassFraction X_default[nX]=reference_X
        "Default value for mass fractions of medium (for initialization)";
      constant ExtraProperty C_default[nC]=fill(0, nC)
        "Default value for trace substances of medium (for initialization)";

      final constant Integer nS=size(substanceNames, 1) "Number of substances";
      constant Integer nX=nS "Number of mass fractions";
      constant Integer nXi=if fixedX then 0 else if reducedX then nS - 1 else nS
        "Number of structurally independent mass fractions (see docu for details)";

      final constant Integer nC=size(extraPropertiesNames, 1)
        "Number of extra (outside of standard mass-balance) transported properties";
      constant Real C_nominal[nC](min=fill(Modelica.Constants.eps, nC)) = 1.0e-6*
        ones(nC) "Default for the nominal values for the extra properties";
      replaceable record FluidConstants =
          Modelica.Media.Interfaces.Types.Basic.FluidConstants
        "Critical, triple, molecular and other standard data of fluid";

      replaceable record ThermodynamicState
        "Minimal variable set that is available as input argument to every medium function"
        extends Modelica.Icons.Record;
      end ThermodynamicState;

      replaceable partial model BaseProperties
        "Base properties (p, d, T, h, u, R, MM and, if applicable, X and Xi) of a medium"
        InputAbsolutePressure p "Absolute pressure of medium";
        InputMassFraction[nXi] Xi(start=reference_X[1:nXi])
          "Structurally independent mass fractions";
        InputSpecificEnthalpy h "Specific enthalpy of medium";
        Density d "Density of medium";
        Temperature T "Temperature of medium";
        MassFraction[nX] X(start=reference_X)
          "Mass fractions (= (component mass)/total mass  m_i/m)";
        SpecificInternalEnergy u "Specific internal energy of medium";
        SpecificHeatCapacity R "Gas constant (of mixture if applicable)";
        MolarMass MM "Molar mass (of mixture or single fluid)";
        ThermodynamicState state
          "Thermodynamic state record for optional functions";
        parameter Boolean preferredMediumStates=false
          "= true if StateSelect.prefer shall be used for the independent property variables of the medium"
          annotation (Evaluate=true, Dialog(tab="Advanced"));
        parameter Boolean standardOrderComponents=true
          "If true, and reducedX = true, the last element of X will be computed from the other ones";
        SI.Conversions.NonSIunits.Temperature_degC T_degC=
            Modelica.SIunits.Conversions.to_degC(T)
          "Temperature of medium in [degC]";
        SI.Conversions.NonSIunits.Pressure_bar p_bar=
            Modelica.SIunits.Conversions.to_bar(p)
          "Absolute pressure of medium in [bar]";

        // Local connector definition, used for equation balancing check
        connector InputAbsolutePressure = input SI.AbsolutePressure
          "Pressure as input signal connector";
        connector InputSpecificEnthalpy = input SI.SpecificEnthalpy
          "Specific enthalpy as input signal connector";
        connector InputMassFraction = input SI.MassFraction
          "Mass fraction as input signal connector";

      equation
        if standardOrderComponents then
          Xi = X[1:nXi];

          if fixedX then
            X = reference_X;
          end if;
          if reducedX and not fixedX then
            X[nX] = 1 - sum(Xi);
          end if;
          for i in 1:nX loop
            assert(X[i] >= -1.e-5 and X[i] <= 1 + 1.e-5, "Mass fraction X[" +
              String(i) + "] = " + String(X[i]) + "of substance " +
              substanceNames[i] + "\nof medium " + mediumName +
              " is not in the range 0..1");
          end for;

        end if;

        assert(p >= 0.0, "Pressure (= " + String(p) + " Pa) of medium \"" +
          mediumName + "\" is negative\n(Temperature = " + String(T) + " K)");
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Rectangle(
                extent={{-100,100},{100,-100}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,255}), Text(
                extent={{-152,164},{152,102}},
                textString="%name",
                lineColor={0,0,255})}), Documentation(info="<html>
<p>
Model <strong>BaseProperties</strong> is a model within package <strong>PartialMedium</strong>
and contains the <strong>declarations</strong> of the minimum number of
variables that every medium model is supposed to support.
A specific medium inherits from model <strong>BaseProperties</strong> and provides
the equations for the basic properties.</p>
<p>
The BaseProperties model contains the following <strong>7+nXi variables</strong>
(nXi is the number of independent mass fractions defined in package
PartialMedium):
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>Variable</strong></td>
      <td><strong>Unit</strong></td>
      <td><strong>Description</strong></td></tr>
  <tr><td>T</td>
      <td>K</td>
      <td>temperature</td></tr>
  <tr><td>p</td>
      <td>Pa</td>
      <td>absolute pressure</td></tr>
  <tr><td>d</td>
      <td>kg/m3</td>
      <td>density</td></tr>
  <tr><td>h</td>
      <td>J/kg</td>
      <td>specific enthalpy</td></tr>
  <tr><td>u</td>
      <td>J/kg</td>
      <td>specific internal energy</td></tr>
  <tr><td>Xi[nXi]</td>
      <td>kg/kg</td>
      <td>independent mass fractions m_i/m</td></tr>
  <tr><td>R</td>
      <td>J/kg.K</td>
      <td>gas constant</td></tr>
  <tr><td>M</td>
      <td>kg/mol</td>
      <td>molar mass</td></tr>
</table>
<p>
In order to implement an actual medium model, one can extend from this
base model and add <strong>5 equations</strong> that provide relations among
these variables. Equations will also have to be added in order to
set all the variables within the ThermodynamicState record state.</p>
<p>
If standardOrderComponents=true, the full composition vector X[nX]
is determined by the equations contained in this base class, depending
on the independent mass fraction vector Xi[nXi].</p>
<p>Additional <strong>2 + nXi</strong> equations will have to be provided
when using the BaseProperties model, in order to fully specify the
thermodynamic conditions. The input connector qualifier applied to
p, h, and nXi indirectly declares the number of missing equations,
permitting advanced equation balance checking by Modelica tools.
Please note that this doesn't mean that the additional equations
should be connection equations, nor that exactly those variables
should be supplied, in order to complete the model.
For further information, see the Modelica.Media User's guide, and
Section 4.7 (Balanced Models) of the Modelica 3.0 specification.</p>
</html>"));
      end BaseProperties;

      replaceable partial function setState_pTX
        "Return thermodynamic state as function of p, T and composition X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      end setState_pTX;

      replaceable partial function setState_phX
        "Return thermodynamic state as function of p, h and composition X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      end setState_phX;

      replaceable partial function setState_psX
        "Return thermodynamic state as function of p, s and composition X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      end setState_psX;

      replaceable partial function setState_dTX
        "Return thermodynamic state as function of d, T and composition X or Xi"
        extends Modelica.Icons.Function;
        input Density d "Density";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      end setState_dTX;

      replaceable partial function setSmoothState
        "Return thermodynamic state so that it smoothly approximates: if x > 0 then state_a else state_b"
        extends Modelica.Icons.Function;
        input Real x "m_flow or dp";
        input ThermodynamicState state_a "Thermodynamic state if x > 0";
        input ThermodynamicState state_b "Thermodynamic state if x < 0";
        input Real x_small(min=0)
          "Smooth transition in the region -x_small < x < x_small";
        output ThermodynamicState state
          "Smooth thermodynamic state for all x (continuous and differentiable)";
        annotation (Documentation(info="<html>
<p>
This function is used to approximate the equation
</p>
<pre>
    state = <strong>if</strong> x &gt; 0 <strong>then</strong> state_a <strong>else</strong> state_b;
</pre>

<p>
by a smooth characteristic, so that the expression is continuous and differentiable:
</p>

<pre>
   state := <strong>smooth</strong>(1, <strong>if</strong> x &gt;  x_small <strong>then</strong> state_a <strong>else</strong>
                      <strong>if</strong> x &lt; -x_small <strong>then</strong> state_b <strong>else</strong> f(state_a, state_b));
</pre>

<p>
This is performed by applying function <strong>Media.Common.smoothStep</strong>(..)
on every element of the thermodynamic state record.
</p>

<p>
If <strong>mass fractions</strong> X[:] are approximated with this function then this can be performed
for all <strong>nX</strong> mass fractions, instead of applying it for nX-1 mass fractions and computing
the last one by the mass fraction constraint sum(X)=1. The reason is that the approximating function has the
property that sum(state.X) = 1, provided sum(state_a.X) = sum(state_b.X) = 1.
This can be shown by evaluating the approximating function in the abs(x) &lt; x_small
region (otherwise state.X is either state_a.X or state_b.X):
</p>

<pre>
    X[1]  = smoothStep(x, X_a[1] , X_b[1] , x_small);
    X[2]  = smoothStep(x, X_a[2] , X_b[2] , x_small);
       ...
    X[nX] = smoothStep(x, X_a[nX], X_b[nX], x_small);
</pre>

<p>
or
</p>

<pre>
    X[1]  = c*(X_a[1]  - X_b[1])  + (X_a[1]  + X_b[1])/2
    X[2]  = c*(X_a[2]  - X_b[2])  + (X_a[2]  + X_b[2])/2;
       ...
    X[nX] = c*(X_a[nX] - X_b[nX]) + (X_a[nX] + X_b[nX])/2;
    c     = (x/x_small)*((x/x_small)^2 - 3)/4
</pre>

<p>
Summing all mass fractions together results in
</p>

<pre>
    sum(X) = c*(sum(X_a) - sum(X_b)) + (sum(X_a) + sum(X_b))/2
           = c*(1 - 1) + (1 + 1)/2
           = 1
</pre>

</html>"));
      end setSmoothState;

      replaceable partial function dynamicViscosity "Return dynamic viscosity"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output DynamicViscosity eta "Dynamic viscosity";
      end dynamicViscosity;

      replaceable partial function thermalConductivity
        "Return thermal conductivity"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output ThermalConductivity lambda "Thermal conductivity";
      end thermalConductivity;

      replaceable function prandtlNumber "Return the Prandtl number"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output PrandtlNumber Pr "Prandtl number";
      algorithm
        Pr := dynamicViscosity(state)*specificHeatCapacityCp(state)/
          thermalConductivity(state);
      end prandtlNumber;

      replaceable partial function pressure "Return pressure"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output AbsolutePressure p "Pressure";
      end pressure;

      replaceable partial function temperature "Return temperature"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output Temperature T "Temperature";
      end temperature;

      replaceable partial function density "Return density"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output Density d "Density";
      end density;

      replaceable partial function specificEnthalpy "Return specific enthalpy"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificEnthalpy h "Specific enthalpy";
      end specificEnthalpy;

      replaceable partial function specificInternalEnergy
        "Return specific internal energy"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificEnergy u "Specific internal energy";
      end specificInternalEnergy;

      replaceable partial function specificEntropy "Return specific entropy"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificEntropy s "Specific entropy";
      end specificEntropy;

      replaceable partial function specificGibbsEnergy
        "Return specific Gibbs energy"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificEnergy g "Specific Gibbs energy";
      end specificGibbsEnergy;

      replaceable partial function specificHelmholtzEnergy
        "Return specific Helmholtz energy"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificEnergy f "Specific Helmholtz energy";
      end specificHelmholtzEnergy;

      replaceable partial function specificHeatCapacityCp
        "Return specific heat capacity at constant pressure"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificHeatCapacity cp
          "Specific heat capacity at constant pressure";
      end specificHeatCapacityCp;

      function heatCapacity_cp = specificHeatCapacityCp
        "Alias for deprecated name";

      replaceable partial function specificHeatCapacityCv
        "Return specific heat capacity at constant volume"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SpecificHeatCapacity cv
          "Specific heat capacity at constant volume";
      end specificHeatCapacityCv;

      function heatCapacity_cv = specificHeatCapacityCv
        "Alias for deprecated name";

      replaceable partial function isentropicExponent
        "Return isentropic exponent"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output IsentropicExponent gamma "Isentropic exponent";
      end isentropicExponent;

      replaceable partial function isentropicEnthalpy
        "Return isentropic enthalpy"
        extends Modelica.Icons.Function;
        input AbsolutePressure p_downstream "Downstream pressure";
        input ThermodynamicState refState "Reference state for entropy";
        output SpecificEnthalpy h_is "Isentropic enthalpy";
        annotation (Documentation(info="<html>
<p>
This function computes an isentropic state transformation:
</p>
<ol>
<li> A medium is in a particular state, refState.</li>
<li> The enthalpy at another state (h_is) shall be computed
     under the assumption that the state transformation from refState to h_is
     is performed with a change of specific entropy ds = 0 and the pressure of state h_is
     is p_downstream and the composition X upstream and downstream is assumed to be the same.</li>
</ol>

</html>"));
      end isentropicEnthalpy;

      replaceable partial function velocityOfSound "Return velocity of sound"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output VelocityOfSound a "Velocity of sound";
      end velocityOfSound;

      replaceable partial function isobaricExpansionCoefficient
        "Return overall the isobaric expansion coefficient beta"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output IsobaricExpansionCoefficient beta "Isobaric expansion coefficient";
        annotation (Documentation(info="<html>
<pre>
beta is defined as  1/v * der(v,T), with v = 1/d, at constant pressure p.
</pre>
</html>"));
      end isobaricExpansionCoefficient;

      function beta = isobaricExpansionCoefficient
        "Alias for isobaricExpansionCoefficient for user convenience";

      replaceable partial function isothermalCompressibility
        "Return overall the isothermal compressibility factor"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output SI.IsothermalCompressibility kappa "Isothermal compressibility";
        annotation (Documentation(info="<html>
<pre>

kappa is defined as - 1/v * der(v,p), with v = 1/d at constant temperature T.

</pre>
</html>"));
      end isothermalCompressibility;

      function kappa = isothermalCompressibility
        "Alias of isothermalCompressibility for user convenience";

      // explicit derivative functions for finite element models
      replaceable partial function density_derp_h
        "Return density derivative w.r.t. pressure at const specific enthalpy"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output DerDensityByPressure ddph "Density derivative w.r.t. pressure";
      end density_derp_h;

      replaceable partial function density_derh_p
        "Return density derivative w.r.t. specific enthalpy at constant pressure"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output DerDensityByEnthalpy ddhp
          "Density derivative w.r.t. specific enthalpy";
      end density_derh_p;

      replaceable partial function density_derp_T
        "Return density derivative w.r.t. pressure at const temperature"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output DerDensityByPressure ddpT "Density derivative w.r.t. pressure";
      end density_derp_T;

      replaceable partial function density_derT_p
        "Return density derivative w.r.t. temperature at constant pressure"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output DerDensityByTemperature ddTp
          "Density derivative w.r.t. temperature";
      end density_derT_p;

      replaceable partial function density_derX
        "Return density derivative w.r.t. mass fraction"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output Density[nX] dddX "Derivative of density w.r.t. mass fraction";
      end density_derX;

      replaceable partial function molarMass
        "Return the molar mass of the medium"
        extends Modelica.Icons.Function;
        input ThermodynamicState state "Thermodynamic state record";
        output MolarMass MM "Mixture molar mass";
      end molarMass;

      replaceable function specificEnthalpy_pTX
        "Return specific enthalpy from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := specificEnthalpy(setState_pTX(
                p,
                T,
                X));
        annotation (inverse(T=temperature_phX(
                      p,
                      h,
                      X)));
      end specificEnthalpy_pTX;

      replaceable function specificEntropy_pTX
        "Return specific enthalpy from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output SpecificEntropy s "Specific entropy";
      algorithm
        s := specificEntropy(setState_pTX(
                p,
                T,
                X));

        annotation (inverse(T=temperature_psX(
                      p,
                      s,
                      X)));
      end specificEntropy_pTX;

      replaceable function density_pTX "Return density from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[:] "Mass fractions";
        output Density d "Density";
      algorithm
        d := density(setState_pTX(
                p,
                T,
                X));
      end density_pTX;

      replaceable function temperature_phX
        "Return temperature from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output Temperature T "Temperature";
      algorithm
        T := temperature(setState_phX(
                p,
                h,
                X));
      end temperature_phX;

      replaceable function density_phX "Return density from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output Density d "Density";
      algorithm
        d := density(setState_phX(
                p,
                h,
                X));
      end density_phX;

      replaceable function temperature_psX
        "Return temperature from p,s, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output Temperature T "Temperature";
      algorithm
        T := temperature(setState_psX(
                p,
                s,
                X));
        annotation (inverse(s=specificEntropy_pTX(
                      p,
                      T,
                      X)));
      end temperature_psX;

      replaceable function density_psX "Return density from p, s, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output Density d "Density";
      algorithm
        d := density(setState_psX(
                p,
                s,
                X));
      end density_psX;

      replaceable function specificEnthalpy_psX
        "Return specific enthalpy from p, s, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := specificEnthalpy(setState_psX(
                p,
                s,
                X));
      end specificEnthalpy_psX;

      type MassFlowRate = SI.MassFlowRate (
          quantity="MassFlowRate." + mediumName,
          min=-1.0e5,
          max=1.e5) "Type for mass flow rate with medium specific attributes";

      // Only for backwards compatibility to version 3.2 (
      // (do not use these definitions in new models, but use Modelica.Media.Interfaces.Choices instead)
      package Choices = Modelica.Media.Interfaces.Choices annotation (obsolete=
            "Use Modelica.Media.Interfaces.Choices");

      annotation (Documentation(info="<html>
<p>
<strong>PartialMedium</strong> is a package and contains all <strong>declarations</strong> for
a medium. This means that constants, models, and functions
are defined that every medium is supposed to support
(some of them are optional). A medium package
inherits from <strong>PartialMedium</strong> and provides the
equations for the medium. The details of this package
are described in
<a href=\"modelica://Modelica.Media.UsersGuide\">Modelica.Media.UsersGuide</a>.
</p>
</html>",   revisions="<html>

</html>"));
    end PartialMedium;

    partial package PartialPureSubstance
      "Base class for pure substances of one chemical substance"
      extends PartialMedium(final reducedX=true, final fixedX=true);

      replaceable function setState_pT "Return thermodynamic state from p and T"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := setState_pTX(
                p,
                T,
                fill(0, 0));
      end setState_pT;

      replaceable function setState_ph "Return thermodynamic state from p and h"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := setState_phX(
                p,
                h,
                fill(0, 0));
      end setState_ph;

      replaceable function setState_ps "Return thermodynamic state from p and s"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := setState_psX(
                p,
                s,
                fill(0, 0));
      end setState_ps;

      replaceable function setState_dT "Return thermodynamic state from d and T"
        extends Modelica.Icons.Function;
        input Density d "Density";
        input Temperature T "Temperature";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := setState_dTX(
                d,
                T,
                fill(0, 0));
      end setState_dT;

      replaceable function density_ph "Return density from p and h"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        output Density d "Density";
      algorithm
        d := density_phX(
                p,
                h,
                fill(0, 0));
      end density_ph;

      replaceable function temperature_ph "Return temperature from p and h"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        output Temperature T "Temperature";
      algorithm
        T := temperature_phX(
                p,
                h,
                fill(0, 0));
      end temperature_ph;

      replaceable function pressure_dT "Return pressure from d and T"
        extends Modelica.Icons.Function;
        input Density d "Density";
        input Temperature T "Temperature";
        output AbsolutePressure p "Pressure";
      algorithm
        p := pressure(setState_dTX(
                d,
                T,
                fill(0, 0)));
      end pressure_dT;

      replaceable function specificEnthalpy_dT
        "Return specific enthalpy from d and T"
        extends Modelica.Icons.Function;
        input Density d "Density";
        input Temperature T "Temperature";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := specificEnthalpy(setState_dTX(
                d,
                T,
                fill(0, 0)));
      end specificEnthalpy_dT;

      replaceable function specificEnthalpy_ps
        "Return specific enthalpy from p and s"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := specificEnthalpy_psX(
                p,
                s,
                fill(0, 0));
      end specificEnthalpy_ps;

      replaceable function temperature_ps "Return temperature from p and s"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        output Temperature T "Temperature";
      algorithm
        T := temperature_psX(
                p,
                s,
                fill(0, 0));
      end temperature_ps;

      replaceable function density_ps "Return density from p and s"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        output Density d "Density";
      algorithm
        d := density_psX(
                p,
                s,
                fill(0, 0));
      end density_ps;

      replaceable function specificEnthalpy_pT
        "Return specific enthalpy from p and T"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := specificEnthalpy_pTX(
                p,
                T,
                fill(0, 0));
      end specificEnthalpy_pT;

      replaceable function density_pT "Return density from p and T"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        output Density d "Density";
      algorithm
        d := density(setState_pTX(
                p,
                T,
                fill(0, 0)));
      end density_pT;

      redeclare replaceable partial model extends BaseProperties(final
          standardOrderComponents=true)
      end BaseProperties;
    end PartialPureSubstance;

    partial package PartialSimpleMedium
      "Medium model with linear dependency of u, h from temperature. All other quantities, especially density, are constant."

      extends Interfaces.PartialPureSubstance(final ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.pT,
          final singleState=true);

      constant SpecificHeatCapacity cp_const
        "Constant specific heat capacity at constant pressure";
      constant SpecificHeatCapacity cv_const
        "Constant specific heat capacity at constant volume";
      constant Density d_const "Constant density";
      constant DynamicViscosity eta_const "Constant dynamic viscosity";
      constant ThermalConductivity lambda_const "Constant thermal conductivity";
      constant VelocityOfSound a_const "Constant velocity of sound";
      constant Temperature T_min "Minimum temperature valid for medium model";
      constant Temperature T_max "Maximum temperature valid for medium model";
      constant Temperature T0=reference_T "Zero enthalpy temperature";
      constant MolarMass MM_const "Molar mass";

      constant FluidConstants[nS] fluidConstants "Fluid constants";

      redeclare record extends ThermodynamicState "Thermodynamic state"
        AbsolutePressure p "Absolute pressure of medium";
        Temperature T "Temperature of medium";
      end ThermodynamicState;

      redeclare replaceable model extends BaseProperties(T(stateSelect=if
              preferredMediumStates then StateSelect.prefer else StateSelect.default),
          p(stateSelect=if preferredMediumStates then StateSelect.prefer else
              StateSelect.default)) "Base properties"
      equation
        assert(T >= T_min and T <= T_max, "
Temperature T (= "   + String(T) + " K) is not
in the allowed range ("   + String(T_min) + " K <= T <= " + String(T_max) + " K)
required from medium model \""   + mediumName + "\".
");

        // h = cp_const*(T-T0);
        h = specificEnthalpy_pTX(
                p,
                T,
                X);
        u = cv_const*(T - T0);
        d = d_const;
        R = 0;
        MM = MM_const;
        state.T = T;
        state.p = p;
        annotation (Documentation(info="<html>
<p>
This is the most simple incompressible medium model, where
specific enthalpy h and specific internal energy u are only
a function of temperature T and all other provided medium
quantities are assumed to be constant.
Note that the (small) influence of the pressure term p/d is neglected.
</p>
</html>"));
      end BaseProperties;

      redeclare function setState_pTX
        "Return thermodynamic state from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := ThermodynamicState(p=p, T=T);
      end setState_pTX;

      redeclare function setState_phX
        "Return thermodynamic state from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := ThermodynamicState(p=p, T=T0 + h/cp_const);
      end setState_phX;

      redeclare replaceable function setState_psX
        "Return thermodynamic state from p, s, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := ThermodynamicState(p=p, T=Modelica.Math.exp(s/cp_const +
          Modelica.Math.log(reference_T)))
          "Here the incompressible limit is used, with cp as heat capacity";
      end setState_psX;

      redeclare function setState_dTX
        "Return thermodynamic state from d, T, and X or Xi"
        extends Modelica.Icons.Function;
        input Density d "Density";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        assert(false,
          "Pressure can not be computed from temperature and density for an incompressible fluid!");
      end setState_dTX;

      redeclare function extends setSmoothState
        "Return thermodynamic state so that it smoothly approximates: if x > 0 then state_a else state_b"
      algorithm
        state := ThermodynamicState(p=Media.Common.smoothStep(
                x,
                state_a.p,
                state_b.p,
                x_small), T=Media.Common.smoothStep(
                x,
                state_a.T,
                state_b.T,
                x_small));
      end setSmoothState;

      redeclare function extends dynamicViscosity "Return dynamic viscosity"

      algorithm
        eta := eta_const;
      end dynamicViscosity;

      redeclare function extends thermalConductivity
        "Return thermal conductivity"

      algorithm
        lambda := lambda_const;
      end thermalConductivity;

      redeclare function extends pressure "Return pressure"

      algorithm
        p := state.p;
      end pressure;

      redeclare function extends temperature "Return temperature"

      algorithm
        T := state.T;
      end temperature;

      redeclare function extends density "Return density"

      algorithm
        d := d_const;
      end density;

      redeclare function extends specificEnthalpy "Return specific enthalpy"

      algorithm
        h := cp_const*(state.T - T0);
      end specificEnthalpy;

      redeclare function extends specificHeatCapacityCp
        "Return specific heat capacity at constant pressure"

      algorithm
        cp := cp_const;
      end specificHeatCapacityCp;

      redeclare function extends specificHeatCapacityCv
        "Return specific heat capacity at constant volume"

      algorithm
        cv := cv_const;
      end specificHeatCapacityCv;

      redeclare function extends isentropicExponent "Return isentropic exponent"

      algorithm
        gamma := cp_const/cv_const;
      end isentropicExponent;

      redeclare function extends velocityOfSound "Return velocity of sound"

      algorithm
        a := a_const;
      end velocityOfSound;

      redeclare function specificEnthalpy_pTX
        "Return specific enthalpy from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[nX] "Mass fractions";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := cp_const*(T - T0);
        annotation (Documentation(info="<html>
<p>
This function computes the specific enthalpy of the fluid, but neglects the (small) influence of the pressure term p/d.
</p>
</html>"));
      end specificEnthalpy_pTX;

      redeclare function temperature_phX
        "Return temperature from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[nX] "Mass fractions";
        output Temperature T "Temperature";
      algorithm
        T := T0 + h/cp_const;
      end temperature_phX;

      redeclare function density_phX "Return density from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[nX] "Mass fractions";
        output Density d "Density";
      algorithm
        d := density(setState_phX(
                p,
                h,
                X));
      end density_phX;

      redeclare function extends specificInternalEnergy
        "Return specific internal energy"
        extends Modelica.Icons.Function;
      algorithm
        //  u := cv_const*(state.T - T0) - reference_p/d_const;
        u := cv_const*(state.T - T0);
        annotation (Documentation(info="<html>
<p>
This function computes the specific internal energy of the fluid, but neglects the (small) influence of the pressure term p/d.
</p>
</html>"));
      end specificInternalEnergy;

      redeclare function extends specificEntropy "Return specific entropy"
        extends Modelica.Icons.Function;
      algorithm
        s := cv_const*Modelica.Math.log(state.T/T0);
      end specificEntropy;

      redeclare function extends specificGibbsEnergy
        "Return specific Gibbs energy"
        extends Modelica.Icons.Function;
      algorithm
        g := specificEnthalpy(state) - state.T*specificEntropy(state);
      end specificGibbsEnergy;

      redeclare function extends specificHelmholtzEnergy
        "Return specific Helmholtz energy"
        extends Modelica.Icons.Function;
      algorithm
        f := specificInternalEnergy(state) - state.T*specificEntropy(state);
      end specificHelmholtzEnergy;

      redeclare function extends isentropicEnthalpy "Return isentropic enthalpy"
      algorithm
        h_is := cp_const*(temperature(refState) - T0);
      end isentropicEnthalpy;

      redeclare function extends isobaricExpansionCoefficient
        "Returns overall the isobaric expansion coefficient beta"
      algorithm
        beta := 0.0;
      end isobaricExpansionCoefficient;

      redeclare function extends isothermalCompressibility
        "Returns overall the isothermal compressibility factor"
      algorithm
        kappa := 0;
      end isothermalCompressibility;

      redeclare function extends density_derp_T
        "Returns the partial derivative of density with respect to pressure at constant temperature"
      algorithm
        ddpT := 0;
      end density_derp_T;

      redeclare function extends density_derT_p
        "Returns the partial derivative of density with respect to temperature at constant pressure"
      algorithm
        ddTp := 0;
      end density_derT_p;

      redeclare function extends density_derX
        "Returns the partial derivative of density with respect to mass fractions at constant pressure and temperature"
      algorithm
        dddX := fill(0, nX);
      end density_derX;

      redeclare function extends molarMass "Return the molar mass of the medium"
      algorithm
        MM := MM_const;
      end molarMass;
    end PartialSimpleMedium;

    package Choices "Types, constants to define menu choices"
      extends Modelica.Icons.Package;

      type IndependentVariables = enumeration(
            T
            "Temperature",
            pT
             "Pressure, Temperature",
            ph
             "Pressure, Specific Enthalpy",
            phX
              "Pressure, Specific Enthalpy, Mass Fraction",
            pTX
              "Pressure, Temperature, Mass Fractions",
            dTX
              "Density, Temperature, Mass Fractions")
        "Enumeration defining the independent variables of a medium";
      annotation (Documentation(info="<html>
<p>
Enumerations and data types for all types of fluids
</p>

<p>
Note: Reference enthalpy might have to be extended with enthalpy of formation.
</p>
</html>"));
    end Choices;

    package Types "Types to be used in fluid models"
      extends Modelica.Icons.Package;

      type AbsolutePressure = SI.AbsolutePressure (
          min=0,
          max=1.e8,
          nominal=1.e5,
          start=1.e5)
        "Type for absolute pressure with medium specific attributes";

      type Density = SI.Density (
          min=0,
          max=1.e5,
          nominal=1,
          start=1) "Type for density with medium specific attributes";

      type DynamicViscosity = SI.DynamicViscosity (
          min=0,
          max=1.e8,
          nominal=1.e-3,
          start=1.e-3)
        "Type for dynamic viscosity with medium specific attributes";

      type EnthalpyFlowRate = SI.EnthalpyFlowRate (
          nominal=1000.0,
          min=-1.0e8,
          max=1.e8) "Type for enthalpy flow rate with medium specific attributes";

      type MassFraction = Real (
          quantity="MassFraction",
          final unit="kg/kg",
          min=0,
          max=1,
          nominal=0.1) "Type for mass fraction with medium specific attributes";

      type MolarMass = SI.MolarMass (
          min=0.001,
          max=0.25,
          nominal=0.032) "Type for molar mass with medium specific attributes";

      type IsentropicExponent = SI.RatioOfSpecificHeatCapacities (
          min=1,
          max=500000,
          nominal=1.2,
          start=1.2)
        "Type for isentropic exponent with medium specific attributes";

      type SpecificEnergy = SI.SpecificEnergy (
          min=-1.0e8,
          max=1.e8,
          nominal=1.e6)
        "Type for specific energy with medium specific attributes";

      type SpecificInternalEnergy = SpecificEnergy
        "Type for specific internal energy with medium specific attributes";

      type SpecificEnthalpy = SI.SpecificEnthalpy (
          min=-1.0e10,
          max=1.e10,
          nominal=1.e6)
        "Type for specific enthalpy with medium specific attributes";

      type SpecificEntropy = SI.SpecificEntropy (
          min=-1.e7,
          max=1.e7,
          nominal=1.e3)
        "Type for specific entropy with medium specific attributes";

      type SpecificHeatCapacity = SI.SpecificHeatCapacity (
          min=0,
          max=1.e7,
          nominal=1.e3,
          start=1.e3)
        "Type for specific heat capacity with medium specific attributes";

      type Temperature = SI.Temperature (
          min=1,
          max=1.e4,
          nominal=300,
          start=288.15) "Type for temperature with medium specific attributes";

      type ThermalConductivity = SI.ThermalConductivity (
          min=0,
          max=500,
          nominal=1,
          start=1)
        "Type for thermal conductivity with medium specific attributes";

      type PrandtlNumber = SI.PrandtlNumber (
          min=1e-3,
          max=1e5,
          nominal=1.0) "Type for Prandtl number with medium specific attributes";

      type VelocityOfSound = SI.Velocity (
          min=0,
          max=1.e5,
          nominal=1000,
          start=1000)
        "Type for velocity of sound with medium specific attributes";

      type ExtraProperty = Real (min=0.0, start=1.0)
        "Type for unspecified, mass-specific property transported by flow";

      type ExtraPropertyFlowRate = Real (unit="kg/s")
        "Type for flow rate of unspecified, mass-specific property";

      type IsobaricExpansionCoefficient = Real (
          min=0,
          max=1.0e8,
          unit="1/K")
        "Type for isobaric expansion coefficient with medium specific attributes";

      type DerDensityByPressure = SI.DerDensityByPressure
        "Type for partial derivative of density with respect to pressure with medium specific attributes";

      type DerDensityByEnthalpy = SI.DerDensityByEnthalpy
        "Type for partial derivative of density with respect to enthalpy with medium specific attributes";

      type DerDensityByTemperature = SI.DerDensityByTemperature
        "Type for partial derivative of density with respect to temperature with medium specific attributes";

      package Basic "The most basic version of a record used in several degrees of detail"
        extends Icons.Package;

        record FluidConstants
          "Critical, triple, molecular and other standard data of fluid"
          extends Modelica.Icons.Record;
          String iupacName
            "Complete IUPAC name (or common name, if non-existent)";
          String casRegistryNumber
            "Chemical abstracts sequencing number (if it exists)";
          String chemicalFormula
            "Chemical formula, (brutto, nomenclature according to Hill";
          String structureFormula "Chemical structure formula";
          MolarMass molarMass "Molar mass";
        end FluidConstants;
      end Basic;
    end Types;
    annotation (Documentation(info="<html>
<p>
This package provides basic interfaces definitions of media models for different
kind of media.
</p>
</html>"));
  end Interfaces;

  package Common "Data structures and fundamental functions for fluid properties"
    extends Modelica.Icons.Package;

    function smoothStep
      "Approximation of a general step, such that the characteristic is continuous and differentiable"
      extends Modelica.Icons.Function;
      input Real x "Abscissa value";
      input Real y1 "Ordinate value for x > 0";
      input Real y2 "Ordinate value for x < 0";
      input Real x_small(min=0) = 1e-5
        "Approximation of step for -x_small <= x <= x_small; x_small > 0 required";
      output Real y "Ordinate value to approximate y = if x > 0 then y1 else y2";
    algorithm
      y := smooth(1, if x > x_small then y1 else if x < -x_small then y2 else if
        abs(x_small) > 0 then (x/x_small)*((x/x_small)^2 - 3)*(y2 - y1)/4 + (y1
         + y2)/2 else (y1 + y2)/2);

      annotation (
        Inline=true,
        smoothOrder=1,
        Documentation(revisions="<html>
<ul>
<li><em>April 29, 2008</em>
    by <a href=\"mailto:Martin.Otter@DLR.de\">Martin Otter</a>:<br>
    Designed and implemented.</li>
<li><em>August 12, 2008</em>
    by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br>
    Minor modification to cover the limit case <code>x_small -> 0</code> without division by zero.</li>
</ul>
</html>",   info="<html>
<p>
This function is used to approximate the equation
</p>
<pre>
    y = <strong>if</strong> x &gt; 0 <strong>then</strong> y1 <strong>else</strong> y2;
</pre>

<p>
by a smooth characteristic, so that the expression is continuous and differentiable:
</p>

<pre>
   y = <strong>smooth</strong>(1, <strong>if</strong> x &gt;  x_small <strong>then</strong> y1 <strong>else</strong>
                 <strong>if</strong> x &lt; -x_small <strong>then</strong> y2 <strong>else</strong> f(y1, y2));
</pre>

<p>
In the region -x_small &lt; x &lt; x_small a 2nd order polynomial is used
for a smooth transition from y1 to y2.
</p>

<p>
If <strong>mass fractions</strong> X[:] are approximated with this function then this can be performed
for all <strong>nX</strong> mass fractions, instead of applying it for nX-1 mass fractions and computing
the last one by the mass fraction constraint sum(X)=1. The reason is that the approximating function has the
property that sum(X) = 1, provided sum(X_a) = sum(X_b) = 1
(and y1=X_a[i], y2=X_b[i]).
This can be shown by evaluating the approximating function in the abs(x) &lt; x_small
region (otherwise X is either X_a or X_b):
</p>

<pre>
    X[1]  = smoothStep(x, X_a[1] , X_b[1] , x_small);
    X[2]  = smoothStep(x, X_a[2] , X_b[2] , x_small);
       ...
    X[nX] = smoothStep(x, X_a[nX], X_b[nX], x_small);
</pre>

<p>
or
</p>

<pre>
    X[1]  = c*(X_a[1]  - X_b[1])  + (X_a[1]  + X_b[1])/2
    X[2]  = c*(X_a[2]  - X_b[2])  + (X_a[2]  + X_b[2])/2;
       ...
    X[nX] = c*(X_a[nX] - X_b[nX]) + (X_a[nX] + X_b[nX])/2;
    c     = (x/x_small)*((x/x_small)^2 - 3)/4
</pre>

<p>
Summing all mass fractions together results in
</p>

<pre>
    sum(X) = c*(sum(X_a) - sum(X_b)) + (sum(X_a) + sum(X_b))/2
           = c*(1 - 1) + (1 + 1)/2
           = 1
</pre>
</html>"));
    end smoothStep;
    annotation (Documentation(info="<html><h4>Package description</h4>
      <p>Package Modelica.Media.Common provides records and functions shared by many of the property sub-packages.
      High accuracy fluid property models share a lot of common structure, even if the actual models are different.
      Common data structures and computations shared by these property models are collected in this library.
   </p>

</html>",   revisions="<html>
      <ul>
      <li>First implemented: <em>July, 2000</em>
      by Hubertus Tummescheit
      for the ThermoFluid Library with help from Jonas Eborn and Falko Jens Wagner
      </li>
      <li>Code reorganization, enhanced documentation, additional functions: <em>December, 2002</em>
      by Hubertus Tummescheit and move to Modelica
                            properties library.</li>
      <li>Inclusion into Modelica.Media: September 2003</li>
      </ul>

      <address>Author: Hubertus Tummescheit,<br>
      Lund University<br>
      Department of Automatic Control<br>
      Box 118, 22100 Lund, Sweden<br>
      email: hubertus@control.lth.se
      </address>
</html>"));
  end Common;

    package Water "Medium models for water"
    extends Modelica.Icons.VariantsPackage;
    import Modelica.Media.Water.ConstantPropertyLiquidWater.simpleWaterConstants;

    package ConstantPropertyLiquidWater
      "Water: Simple liquid water medium (incompressible, constant data)"

      //   redeclare record extends FluidConstants
      //   end FluidConstants;

      constant Modelica.Media.Interfaces.Types.Basic.FluidConstants[1]
        simpleWaterConstants(
        each chemicalFormula="H2O",
        each structureFormula="H2O",
        each casRegistryNumber="7732-18-5",
        each iupacName="oxidane",
        each molarMass=0.018015268);

      extends Interfaces.PartialSimpleMedium(
        mediumName="SimpleLiquidWater",
        cp_const=4184,
        cv_const=4184,
        d_const=995.586,
        eta_const=1.e-3,
        lambda_const=0.598,
        a_const=1484,
        T_min=Cv.from_degC(-1),
        T_max=Cv.from_degC(130),
        T0=273.15,
        MM_const=0.018015268,
        fluidConstants=simpleWaterConstants);

      annotation (Documentation(info="<html>

</html>"));
    end ConstantPropertyLiquidWater;
    annotation (Documentation(info="<html>
<p>This package contains different medium models for water:</p>
<ul>
<li><strong>ConstantPropertyLiquidWater</strong><br>
    Simple liquid water medium (incompressible, constant data).</li>
<li><strong>IdealSteam</strong><br>
    Steam water medium as ideal gas from Media.IdealGases.SingleGases.H2O</li>
<li><strong>WaterIF97 derived models</strong><br>
    High precision water model according to the IAPWS/IF97 standard
    (liquid, steam, two phase region). Models with different independent
    variables are provided as well as models valid only
    for particular regions. The <strong>WaterIF97_ph</strong> model is valid
    in all regions and is the recommended one to use.</li>
</ul>
<h4>Overview of WaterIF97 derived water models</h4>
<p>
The WaterIF97 models calculate medium properties
for water in the <strong>liquid</strong>, <strong>gas</strong> and <strong>two phase</strong> regions
according to the IAPWS/IF97 standard, i.e., the accepted industrial standard
and best compromise between accuracy and computation time.
It has been part of the ThermoFluid Modelica library and been extended,
reorganized and documented to become part of the Modelica Standard library.</p>
<p>An important feature that distinguishes this implementation of the IF97 steam property standard
is that this implementation has been explicitly designed to work well in dynamic simulations. Computational
performance has been of high importance. This means that there often exist several ways to get the same result
from different functions if one of the functions is called often but can be optimized for that purpose.
</p>
<p>Three variable pairs can be the independent variables of the model:
</p>
<ol>
<li>Pressure <strong>p</strong> and specific enthalpy <strong>h</strong> are
    the most natural choice for general applications.
    This is the recommended choice for most general purpose
    applications, in particular for power plants.</li>
<li>Pressure <strong>p</strong> and temperature <strong>T</strong> are the most natural
    choice for applications where water is always in the same phase,
    both for liquid water and steam.</li>
<li>Density <strong>d</strong> and temperature <strong>T</strong> are explicit
    variables of the Helmholtz function in the near-critical
    region and can be the best choice for applications with
    super-critical or near-critical states.</li>
</ol>
<p>
The following quantities are always computed in Medium.BaseProperties:
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>Variable</strong></td>
      <td><strong>Unit</strong></td>
      <td><strong>Description</strong></td></tr>
  <tr><td>T</td>
      <td>K</td>
      <td>temperature</td></tr>
  <tr><td>u</td>
      <td>J/kg</td>
      <td>specific internal energy</td></tr>
  <tr><td>d</td>
      <td>kg/m^3</td>
      <td>density</td></tr>
  <tr><td>p</td>
      <td>Pa</td>
      <td>pressure</td></tr>
  <tr><td>h</td>
      <td>J/kg</td>
      <td>specific enthalpy</td></tr>
</table>
<p>
In some cases additional medium properties are needed.
A component that needs these optional properties has to call
one of the following functions:
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>Function call</strong></td>
      <td><strong>Unit</strong></td>
      <td><strong>Description</strong></td></tr>
  <tr><td>Medium.dynamicViscosity(medium.state)</td>
      <td>Pa.s</td>
      <td>dynamic viscosity</td></tr>
  <tr><td>Medium.thermalConductivity(medium.state)</td>
      <td>W/(m.K)</td>
      <td>thermal conductivity</td></tr>
  <tr><td>Medium.prandtlNumber(medium.state)</td>
      <td>1</td>
      <td>Prandtl number</td></tr>
  <tr><td>Medium.specificEntropy(medium.state)</td>
      <td>J/(kg.K)</td>
      <td>specific entropy</td></tr>
  <tr><td>Medium.heatCapacity_cp(medium.state)</td>
      <td>J/(kg.K)</td>
      <td>specific heat capacity at constant pressure</td></tr>
  <tr><td>Medium.heatCapacity_cv(medium.state)</td>
      <td>J/(kg.K)</td>
      <td>specific heat capacity at constant density</td></tr>
  <tr><td>Medium.isentropicExponent(medium.state)</td>
      <td>1</td>
      <td>isentropic exponent</td></tr>
  <tr><td>Medium.isentropicEnthalpy(pressure, medium.state)</td>
      <td>J/kg</td>
      <td>isentropic enthalpy</td></tr>
  <tr><td>Medium.velocityOfSound(medium.state)</td>
      <td>m/s</td>
      <td>velocity of sound</td></tr>
  <tr><td>Medium.isobaricExpansionCoefficient(medium.state)</td>
      <td>1/K</td>
      <td>isobaric expansion coefficient</td></tr>
  <tr><td>Medium.isothermalCompressibility(medium.state)</td>
      <td>1/Pa</td>
      <td>isothermal compressibility</td></tr>
  <tr><td>Medium.density_derp_h(medium.state)</td>
      <td>kg/(m3.Pa)</td>
      <td>derivative of density by pressure at constant enthalpy</td></tr>
  <tr><td>Medium.density_derh_p(medium.state)</td>
      <td>kg2/(m3.J)</td>
      <td>derivative of density by enthalpy at constant pressure</td></tr>
  <tr><td>Medium.density_derp_T(medium.state)</td>
      <td>kg/(m3.Pa)</td>
      <td>derivative of density by pressure at constant temperature</td></tr>
  <tr><td>Medium.density_derT_p(medium.state)</td>
      <td>kg/(m3.K)</td>
      <td>derivative of density by temperature at constant pressure</td></tr>
  <tr><td>Medium.density_derX(medium.state)</td>
      <td>kg/m3</td>
      <td>derivative of density by mass fraction</td></tr>
  <tr><td>Medium.molarMass(medium.state)</td>
      <td>kg/mol</td>
      <td>molar mass</td></tr>
</table>
<p>More details are given in
<a href=\"modelica://Modelica.Media.UsersGuide.MediumUsage.OptionalProperties\">
Modelica.Media.UsersGuide.MediumUsage.OptionalProperties</a>.

Many additional optional functions are defined to compute properties of
saturated media, either liquid (bubble point) or vapour (dew point).
The argument to such functions is a SaturationProperties record, which can be
set starting from either the saturation pressure or the saturation temperature.
With reference to a model defining a pressure p, a temperature T, and a
SaturationProperties record sat, the following functions are provided:
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><td><strong>Function call</strong></td>
      <td><strong>Unit</strong></td>
      <td><strong>Description</strong></td></tr>
  <tr><td>Medium.saturationPressure(T)</td>
      <td>Pa</td>
      <td>Saturation pressure at temperature T</td></tr>
  <tr><td>Medium.saturationTemperature(p)</td>
      <td>K</td>
      <td>Saturation temperature at pressure p</td></tr>
  <tr><td>Medium.saturationTemperature_derp(p)</td>
      <td>K/Pa</td>
      <td>Derivative of saturation temperature with respect to pressure</td></tr>
  <tr><td>Medium.bubbleEnthalpy(sat)</td>
      <td>J/kg</td>
      <td>Specific enthalpy at bubble point</td></tr>
  <tr><td>Medium.dewEnthalpy(sat)</td>
      <td>J/kg</td>
      <td>Specific enthalpy at dew point</td></tr>
  <tr><td>Medium.bubbleEntropy(sat)</td>
      <td>J/(kg.K)</td>
      <td>Specific entropy at bubble point</td></tr>
  <tr><td>Medium.dewEntropy(sat)</td>
      <td>J/(kg.K)</td>
      <td>Specific entropy at dew point</td></tr>
  <tr><td>Medium.bubbleDensity(sat)</td>
      <td>kg/m3</td>
      <td>Density at bubble point</td></tr>
  <tr><td>Medium.dewDensity(sat)</td>
      <td>kg/m3</td>
      <td>Density at dew point</td></tr>
  <tr><td>Medium.dBubbleDensity_dPressure(sat)</td>
      <td>kg/(m3.Pa)</td>
      <td>Derivative of density at bubble point with respect to pressure</td></tr>
  <tr><td>Medium.dDewDensity_dPressure(sat)</td>
      <td>kg/(m3.Pa)</td>
      <td>Derivative of density at dew point with respect to pressure</td></tr>
  <tr><td>Medium.dBubbleEnthalpy_dPressure(sat)</td>
      <td>J/(kg.Pa)</td>
      <td>Derivative of specific enthalpy at bubble point with respect to pressure</td></tr>
  <tr><td>Medium.dDewEnthalpy_dPressure(sat)</td>
      <td>J/(kg.Pa)</td>
      <td>Derivative of specific enthalpy at dew point with respect to pressure</td></tr>
  <tr><td>Medium.surfaceTension(sat)</td>
      <td>N/m</td>
      <td>Surface tension between liquid and vapour phase</td></tr>
</table>
<p>Details on usage and some examples are given in:
<a href=\"modelica://Modelica.Media.UsersGuide.MediumUsage.TwoPhase\">
Modelica.Media.UsersGuide.MediumUsage.TwoPhase</a>.
</p>
<p>Many further properties can be computed. Using the well-known Bridgman's Tables,
all first partial derivatives of the standard thermodynamic variables can be computed easily.
</p>
<p>
The documentation of the IAPWS/IF97 steam properties can be freely
distributed with computer implementations and are included here
(in directory Modelica/Resources/Documentation/Media/Water/IF97documentation):
</p>
<ul>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/IF97.pdf\">IF97.pdf</a> The standards document for the main part of the IF97.</li>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/Back3.pdf\">Back3.pdf</a> The backwards equations for region 3.</li>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/crits.pdf\">crits.pdf</a> The critical point data.</li>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/meltsub.pdf\">meltsub.pdf</a> The melting- and sublimation line formulation (not implemented)</li>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/surf.pdf\">surf.pdf</a> The surface tension standard definition</li>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/thcond.pdf\">thcond.pdf</a> The thermal conductivity standard definition</li>
<li><a href=\"modelica://Modelica/Resources/Documentation/Media/Water/IF97documentation/visc.pdf\">visc.pdf</a> The viscosity standard definition</li>
</ul>
</html>"));
    end Water;
  annotation (preferredView="info",Documentation(info="<html>
<p>
This library contains <a href=\"modelica://Modelica.Media.Interfaces\">interface</a>
definitions for media and the following <strong>property</strong> models for
single and multiple substance fluids with one and multiple phases:
</p>
<ul>
<li> <a href=\"modelica://Modelica.Media.IdealGases\">Ideal gases:</a><br>
     1241 high precision gas models based on the
     NASA Glenn coefficients, plus ideal gas mixture models based
     on the same data.</li>
<li> <a href=\"modelica://Modelica.Media.Water\">Water models:</a><br>
     ConstantPropertyLiquidWater, WaterIF97 (high precision
     water model according to the IAPWS/IF97 standard)</li>
<li> <a href=\"modelica://Modelica.Media.Air\">Air models:</a><br>
     SimpleAir, DryAirNasa, ReferenceAir, MoistAir, ReferenceMoistAir.</li>
<li> <a href=\"modelica://Modelica.Media.Incompressible\">
     Incompressible media:</a><br>
     TableBased incompressible fluid models (properties are defined by tables rho(T),
     HeatCapacity_cp(T), etc.)</li>
<li> <a href=\"modelica://Modelica.Media.CompressibleLiquids\">
     Compressible liquids:</a><br>
     Simple liquid models with linear compressibility</li>
<li> <a href=\"modelica://Modelica.Media.R134a\">Refrigerant Tetrafluoroethane (R134a)</a>.</li>
</ul>
<p>
The following parts are useful, when newly starting with this library:</p>
<ul>
<li> <a href=\"modelica://Modelica.Media.UsersGuide\">Modelica.Media.UsersGuide</a>.</li>
<li> <a href=\"modelica://Modelica.Media.UsersGuide.MediumUsage\">Modelica.Media.UsersGuide.MediumUsage</a>
     describes how to use a medium model in a component model.</li>
<li> <a href=\"modelica://Modelica.Media.UsersGuide.MediumDefinition\">
     Modelica.Media.UsersGuide.MediumDefinition</a>
     describes how a new fluid medium model has to be implemented.</li>
<li> <a href=\"modelica://Modelica.Media.UsersGuide.ReleaseNotes\">Modelica.Media.UsersGuide.ReleaseNotes</a>
     summarizes the changes of the library releases.</li>
<li> <a href=\"modelica://Modelica.Media.Examples\">Modelica.Media.Examples</a>
     contains examples that demonstrate the usage of this library.</li>
</ul>
<p>
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>",   revisions="<html>
<ul>
<li><em>February 01, 2017</em> by Thomas Beutlich:<br/>
    Fixed data errors of the NASA Glenn coefficients in some ideal gases (CH2, CH3, CH3OOH, C2CL2, C2CL4, C2CL6, C2HCL, C2HCL3, CH2CO_ketene, O_CH_2O, HO_CO_2OH, CH2BrminusCOOH, C2H3CL, CH2CLminusCOOH, HO2, HO2minus, OD, ODminus), see <a href=\"https://github.com/modelica/ModelicaStandardLibrary/issues/1922\">#1922</a></li>
<li><em>May 16, 2013</em> by Stefan Wischhusen (XRG Simulation):<br/>
    Added new media models Air.ReferenceMoistAir, Air.ReferenceAir, R134a.</li>
<li><em>May 25, 2011</em> by Francesco Casella:<br/>Added min/max attributes to Water, TableBased, MixtureGasNasa, SimpleAir and MoistAir local types.</li>
<li><em>May 25, 2011</em> by Stefan Wischhusen:<br/>Added individual settings for polynomial fittings of properties.</li>
</ul>
</html>"),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Line(
            points = {{-76,-80},{-62,-30},{-32,40},{4,66},{48,66},{73,45},{62,-8},{48,-50},{38,-80}},
            color={64,64,64},
            smooth=Smooth.Bezier),
          Line(
            points={{-40,20},{68,20}},
            color={175,175,175}),
          Line(
            points={{-40,20},{-44,88},{-44,88}},
            color={175,175,175}),
          Line(
            points={{68,20},{86,-58}},
            color={175,175,175}),
          Line(
            points={{-60,-28},{56,-28}},
            color={175,175,175}),
          Line(
            points={{-60,-28},{-74,84},{-74,84}},
            color={175,175,175}),
          Line(
            points={{56,-28},{70,-80}},
            color={175,175,175}),
          Line(
            points={{-76,-80},{38,-80}},
            color={175,175,175}),
          Line(
            points={{-76,-80},{-94,-16},{-94,-16}},
            color={175,175,175})}));
  end Media;

  package Thermal "Library of thermal system components to model heat transfer and simple thermo-fluid pipe flow"
    extends Modelica.Icons.Package;

    package HeatTransfer "Library of 1-dimensional heat transfer with lumped elements"
      extends Modelica.Icons.Package;

      package Sensors "Thermal sensors"
        extends Modelica.Icons.SensorsPackage;

        model HeatFlowSensor "Heat flow rate sensor"
          extends Modelica.Icons.RotationalSensor;
          Modelica.Blocks.Interfaces.RealOutput Q_flow(unit="W")
            "Heat flow from port_a to port_b as output signal" annotation (Placement(
                transformation(
                origin={0,-100},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Interfaces.HeatPort_a port_a annotation (Placement(transformation(extent={{
                    -110,-10},{-90,10}})));
          Interfaces.HeatPort_b port_b annotation (Placement(transformation(extent={{
                    90,-10},{110,10}})));
        equation
          port_a.T = port_b.T;
          port_a.Q_flow + port_b.Q_flow = 0;
          Q_flow = port_a.Q_flow;
          annotation (
            Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                    100,100}}), graphics={
                Line(points={{-70,0},{-95,0}}, color={191,0,0}),
                Line(points={{0,-70},{0,-90}}, color={0,0,127}),
                Line(points={{94,0},{69,0}}, color={191,0,0})}),
            Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                    100,100}}), graphics={
                Text(
                  extent={{5,-86},{116,-110}},
                  textString="Q_flow"),
                Line(points={{-70,0},{-90,0}}, color={191,0,0}),
                Line(points={{69,0},{90,0}}, color={191,0,0}),
                Line(points={{0,-70},{0,-90}}, color={0,0,127}),
                Text(
                  extent={{-150,125},{150,85}},
                  textString="%name",
                  lineColor={0,0,255})}),
            Documentation(info="<html>
<p>
This model is capable of monitoring the heat flow rate flowing through
this component. The sensed value of heat flow rate is the amount that
passes through this sensor while keeping the temperature drop across the
sensor zero.  This is an ideal model so it does not absorb any energy
and it has no direct effect on the thermal response of a system it is included in.
The output signal is positive, if the heat flows from port_a to port_b.
</p>
</html>"));
        end HeatFlowSensor;
        annotation (Documentation(info="<html>

</html>"));
      end Sensors;

      package Sources "Thermal sources"
      extends Modelica.Icons.SourcesPackage;

        model FixedTemperature "Fixed temperature boundary condition in Kelvin"

          parameter Modelica.SIunits.Temperature T "Fixed temperature at port";
          Interfaces.HeatPort_b port annotation (Placement(transformation(extent={{90,
                    -10},{110,10}})));
        equation
          port.T = T;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                    100,100}}), graphics={
                Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),
                Text(
                  extent={{-150,-110},{150,-140}},
                  textString="T=%T"),
                Rectangle(
                  extent={{-100,100},{100,-100}},
                  pattern=LinePattern.None,
                  fillColor={159,159,223},
                  fillPattern=FillPattern.Backward),
                Text(
                  extent={{0,0},{-100,-100}},
                  textString="K"),
                Line(
                  points={{-52,0},{56,0}},
                  color={191,0,0},
                  thickness=0.5),
                Polygon(
                  points={{50,-20},{50,20},{90,0},{50,-20}},
                  lineColor={191,0,0},
                  fillColor={191,0,0},
                  fillPattern=FillPattern.Solid)}),
            Documentation(info="<html>
<p>
This model defines a fixed temperature T at its port in Kelvin,
i.e., it defines a fixed temperature as a boundary condition.
</p>
</html>"),     Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                    100,100}}), graphics={
                Rectangle(
                  extent={{-100,100},{100,-101}},
                  pattern=LinePattern.None,
                  fillColor={159,159,223},
                  fillPattern=FillPattern.Backward),
                Line(
                  points={{-52,0},{56,0}},
                  color={191,0,0},
                  thickness=0.5),
                Text(
                  extent={{0,0},{-100,-100}},
                  textString="K"),
                Polygon(
                  points={{52,-20},{52,20},{90,0},{52,-20}},
                  lineColor={191,0,0},
                  fillColor={191,0,0},
                  fillPattern=FillPattern.Solid)}));
        end FixedTemperature;
        annotation (Documentation(info="<html>

</html>"));
      end Sources;

      package Interfaces "Connectors and partial models"
        extends Modelica.Icons.InterfacesPackage;

        partial connector HeatPort "Thermal port for 1-dim. heat transfer"
          Modelica.SIunits.Temperature T "Port temperature";
          flow Modelica.SIunits.HeatFlowRate Q_flow
            "Heat flow rate (positive if flowing from outside into the component)";
          annotation (Documentation(info="<html>

</html>"));
        end HeatPort;

        connector HeatPort_a
          "Thermal port for 1-dim. heat transfer (filled rectangular icon)"

          extends HeatPort;

          annotation(defaultComponentName = "port_a",
            Documentation(info="<html>
<p>This connector is used for 1-dimensional heat flow between components.
The variables in the connector are:</p>
<pre>
   T       Temperature in [Kelvin].
   Q_flow  Heat flow rate in [Watt].
</pre>
<p>According to the Modelica sign convention, a <strong>positive</strong> heat flow
rate <strong>Q_flow</strong> is considered to flow <strong>into</strong> a component. This
convention has to be used whenever this connector is used in a model
class.</p>
<p>Note, that the two connector classes <strong>HeatPort_a</strong> and
<strong>HeatPort_b</strong> are identical with the only exception of the different
<strong>icon layout</strong>.</p></html>"),     Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                    100,100}}), graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={191,0,0},
                  fillColor={191,0,0},
                  fillPattern=FillPattern.Solid)}),
            Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                    {100,100}}), graphics={Rectangle(
                  extent={{-50,50},{50,-50}},
                  lineColor={191,0,0},
                  fillColor={191,0,0},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-120,120},{100,60}},
                  lineColor={191,0,0},
                  textString="%name")}));
        end HeatPort_a;

        connector HeatPort_b
          "Thermal port for 1-dim. heat transfer (unfilled rectangular icon)"

          extends HeatPort;

          annotation(defaultComponentName = "port_b",
            Documentation(info="<html>
<p>This connector is used for 1-dimensional heat flow between components.
The variables in the connector are:</p>
<pre>
   T       Temperature in [Kelvin].
   Q_flow  Heat flow rate in [Watt].
</pre>
<p>According to the Modelica sign convention, a <strong>positive</strong> heat flow
rate <strong>Q_flow</strong> is considered to flow <strong>into</strong> a component. This
convention has to be used whenever this connector is used in a model
class.</p>
<p>Note, that the two connector classes <strong>HeatPort_a</strong> and
<strong>HeatPort_b</strong> are identical with the only exception of the different
<strong>icon layout</strong>.</p></html>"),     Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                    {100,100}}), graphics={Rectangle(
                  extent={{-50,50},{50,-50}},
                  lineColor={191,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid), Text(
                  extent={{-100,120},{120,60}},
                  lineColor={191,0,0},
                  textString="%name")}),
            Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                    100,100}}), graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={191,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}));
        end HeatPort_b;
        annotation (Documentation(info="<html>

</html>"));
      end Interfaces;
      annotation (
         Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}), graphics={
          Polygon(
            origin = {13.758,27.517},
            lineColor = {128,128,128},
            fillColor = {192,192,192},
            fillPattern = FillPattern.Solid,
            points = {{-54,-6},{-61,-7},{-75,-15},{-79,-24},{-80,-34},{-78,-42},{-73,-49},{-64,-51},{-57,-51},{-47,-50},{-41,-43},{-38,-35},{-40,-27},{-40,-20},{-42,-13},{-47,-7},{-54,-5},{-54,-6}}),
        Polygon(
            origin = {13.758,27.517},
            fillColor = {160,160,164},
            fillPattern = FillPattern.Solid,
            points = {{-75,-15},{-79,-25},{-80,-34},{-78,-42},{-72,-49},{-64,-51},{-57,-51},{-47,-50},{-57,-47},{-65,-45},{-71,-40},{-74,-33},{-76,-23},{-75,-15},{-75,-15}}),
          Polygon(
            origin = {13.758,27.517},
            lineColor = {160,160,164},
            fillColor = {192,192,192},
            fillPattern = FillPattern.Solid,
            points = {{39,-6},{32,-7},{18,-15},{14,-24},{13,-34},{15,-42},{20,-49},{29,-51},{36,-51},{46,-50},{52,-43},{55,-35},{53,-27},{53,-20},{51,-13},{46,-7},{39,-5},{39,-6}}),
          Polygon(
            origin = {13.758,27.517},
            fillColor = {160,160,164},
            fillPattern = FillPattern.Solid,
            points = {{18,-15},{14,-25},{13,-34},{15,-42},{21,-49},{29,-51},{36,-51},{46,-50},{36,-47},{28,-45},{22,-40},{19,-33},{17,-23},{18,-15},{18,-15}}),
          Polygon(
            origin = {13.758,27.517},
            lineColor = {191,0,0},
            fillColor = {191,0,0},
            fillPattern = FillPattern.Solid,
            points = {{-9,-23},{-9,-10},{18,-17},{-9,-23}}),
          Line(
            origin = {13.758,27.517},
            points = {{-41,-17},{-9,-17}},
            color = {191,0,0},
            thickness = 0.5),
          Line(
            origin = {13.758,27.517},
            points = {{-17,-40},{15,-40}},
            color = {191,0,0},
            thickness = 0.5),
          Polygon(
            origin = {13.758,27.517},
            lineColor = {191,0,0},
            fillColor = {191,0,0},
            fillPattern = FillPattern.Solid,
            points = {{-17,-46},{-17,-34},{-40,-40},{-17,-46}})}),
                                Documentation(info="<html>
<p>
This package contains components to model <strong>1-dimensional heat transfer</strong>
with lumped elements. This allows especially to model heat transfer in
machines provided the parameters of the lumped elements, such as
the heat capacity of a part, can be determined by measurements
(due to the complex geometries and many materials used in machines,
calculating the lumped element parameters from some basic analytic
formulas is usually not possible).
</p>
<p>
Example models how to use this library are given in subpackage <strong>Examples</strong>.<br>
For a first simple example, see <strong>Examples.TwoMasses</strong> where two masses
with different initial temperatures are getting in contact to each
other and arriving after some time at a common temperature.<br>
<strong>Examples.ControlledTemperature</strong> shows how to hold a temperature
within desired limits by switching on and off an electric resistor.<br>
A more realistic example is provided in <strong>Examples.Motor</strong> where the
heating of an electrical motor is modelled, see the following screen shot
of this example:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Thermal/HeatTransfer/driveWithHeatTransfer.png\" alt=\"driveWithHeatTransfer\">
</p>

<p>
The <strong>filled</strong> and <strong>non-filled red squares</strong> at the left and
right side of a component represent <strong>thermal ports</strong> (connector HeatPort).
Drawing a line between such squares means that they are thermally connected.
The variables of a HeatPort connector are the temperature <strong>T</strong> at the port
and the heat flow rate <strong>Q_flow</strong> flowing into the component (if Q_flow is positive,
the heat flows into the element, otherwise it flows out of the element):
</p>
<pre>   Modelica.SIunits.Temperature  T  \"absolute temperature at port in Kelvin\";
   Modelica.SIunits.HeatFlowRate Q_flow  \"flow rate at the port in Watt\";
</pre>
<p>
Note, that all temperatures of this package, including initial conditions,
are given in Kelvin. For convenience, in subpackages <strong>HeatTransfer.Celsius</strong>,
 <strong>HeatTransfer.Fahrenheit</strong> and <strong>HeatTransfer.Rankine</strong> components are provided such that source and
sensor information is available in degree Celsius, degree Fahrenheit, or degree Rankine,
respectively. Additionally, in package <strong>SIunits.Conversions</strong> conversion
functions between the units Kelvin and Celsius, Fahrenheit, Rankine are
provided. These functions may be used in the following way:
</p>
<pre>  <strong>import</strong> SI=Modelica.SIunits;
  <strong>import</strong> Modelica.SIunits.Conversions.*;
     ...
  <strong>parameter</strong> SI.Temperature T = from_degC(25);  // convert 25 degree Celsius to Kelvin
</pre>

<p>
There are several other components available, such as AxialConduction (discretized PDE in
axial direction), which have been temporarily removed from this library. The reason is that
these components reference material properties, such as thermal conductivity, and currently
the Modelica design group is discussing a general scheme to describe material properties.
</p>
<p>
For technical details in the design of this library, see the following reference:<br>
<strong>Michael Tiller (2001)</strong>: <a href=\"http://www.amazon.de\">
Introduction to Physical Modeling with Modelica</a>.
Kluwer Academic Publishers Boston.
</p>
<p>
<strong>Acknowledgements:</strong><br>
Several helpful remarks from the following persons are acknowledged:
John Batteh, Ford Motors, Dearborn, U.S.A;
<a href=\"https://www.haumer.at/\">Anton Haumer</a>, Technical Consulting &amp; Electrical Engineering, Germany;
Ludwig Marvan, VA TECH ELIN EBG Elektronik GmbH, Wien, Austria;
Hans Olsson, Dassault Syst&egrave;mes AB, Sweden;
Hubertus Tummescheit, Lund Institute of Technology, Lund, Sweden.
</p>
<dl>
  <dt><strong>Main Authors:</strong></dt>
  <dd>
  <p>
  <a href=\"https://www.haumer.at/\">Anton Haumer</a><br>
  Technical Consulting &amp; Electrical Engineering<br>
  D-93049 Regensburg, Germany<br>
  email: <a href=\"mailto:a.haumer@haumer.at\">a.haumer@haumer.at</a>
</p>
  </dd>
</dl>
<p>
Copyright &copy; 2001-2019, Modelica Association and contributors
</p>
</html>",     revisions="<html>
<ul>
<li><em>July 15, 2002</em>
       by Michael Tiller, <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Nikolaus Sch&uuml;rmann:<br>
       Implemented.
</li>
<li><em>June 13, 2005</em>
       by <a href=\"https://www.haumer.at/\">Anton Haumer</a><br>
       Refined placing of connectors (cosmetic).<br>
       Refined all Examples; removed Examples.FrequencyInverter, introducing Examples.Motor<br>
       Introduced temperature dependent correction (1 + alpha*(T - T_ref)) in Fixed/PrescribedHeatFlow<br>
</li>
  <li> v1.1.1 2007/11/13 Anton Haumer<br>
       components moved to sub-packages</li>
  <li> v1.2.0 2009/08/26 Anton Haumer<br>
       added component ThermalCollector</li>

</ul>
</html>"));
    end HeatTransfer;
    annotation (
     Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
      Line(
      origin={-47.5,11.6667},
      points={{-2.5,-91.6667},{17.5,-71.6667},{-22.5,-51.6667},{17.5,-31.6667},{-22.5,-11.667},{17.5,8.3333},{-2.5,28.3333},{-2.5,48.3333}},
        smooth=Smooth.Bezier),
      Polygon(
      origin={-50.0,68.333},
      pattern=LinePattern.None,
      fillPattern=FillPattern.Solid,
        points={{0.0,21.667},{-10.0,-8.333},{10.0,-8.333}}),
      Line(
      origin={2.5,11.6667},
      points={{-2.5,-91.6667},{17.5,-71.6667},{-22.5,-51.6667},{17.5,-31.6667},{-22.5,-11.667},{17.5,8.3333},{-2.5,28.3333},{-2.5,48.3333}},
        smooth=Smooth.Bezier),
      Polygon(
      origin={0.0,68.333},
      pattern=LinePattern.None,
      fillPattern=FillPattern.Solid,
        points={{0.0,21.667},{-10.0,-8.333},{10.0,-8.333}}),
      Line(
      origin={52.5,11.6667},
      points={{-2.5,-91.6667},{17.5,-71.6667},{-22.5,-51.6667},{17.5,-31.6667},{-22.5,-11.667},{17.5,8.3333},{-2.5,28.3333},{-2.5,48.3333}},
        smooth=Smooth.Bezier),
      Polygon(
      origin={50.0,68.333},
      pattern=LinePattern.None,
      fillPattern=FillPattern.Solid,
        points={{0.0,21.667},{-10.0,-8.333},{10.0,-8.333}})}),
      Documentation(info="<html>
<p>
This package contains libraries to model heat transfer
and fluid heat flow.
</p>
</html>"));
  end Thermal;

  package Math "Library of mathematical functions (e.g., sin, cos) and of functions operating on vectors and matrices"
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
                100,100}}), graphics={Line(points={{-80,80},{-88,80}}, color={95,95,95}),
              Line(points={{-80,-80},{-88,-80}}, color={95,95,95}),Line(
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

  function cos "Cosine"
    extends Modelica.Math.Icons.AxisLeft;
    input SI.Angle u;
    output Real y;

  external "builtin" y = cos(u);
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
          Line(points={{-80,80},{-74.4,78.1},{-68.7,72.3},{-63.1,63},{-56.7,48.7},
                {-48.6,26.6},{-29.3,-32.5},{-22.1,-51.7},{-15.7,-65.3},{-10.1,-73.8},
                {-4.42,-78.8},{1.21,-79.9},{6.83,-77.1},{12.5,-70.6},{18.1,-60.6},
                {24.5,-45.7},{32.6,-23},{50.3,31.3},{57.5,50.7},{63.9,64.6},{69.5,
                73.4},{75.2,78.6},{80,80}}),
          Text(
            extent={{-36,82},{36,34}},
            lineColor={192,192,192},
            textString="cos")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Text(
              extent={{-103,72},{-83,88}},
              textString="1",
              lineColor={0,0,255}),Text(
              extent={{-103,-72},{-83,-88}},
              textString="-1",
              lineColor={0,0,255}),Text(
              extent={{70,25},{90,5}},
              textString="2*pi",
              lineColor={0,0,255}),Line(points={{-100,0},{84,0}}, color={95,95,95}),
            Polygon(
              points={{98,0},{82,6},{82,-6},{98,0}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-80,80},{-74.4,78.1},{-68.7,72.3},{-63.1,63},{-56.7,48.7},
              {-48.6,26.6},{-29.3,-32.5},{-22.1,-51.7},{-15.7,-65.3},{-10.1,-73.8},
              {-4.42,-78.8},{1.21,-79.9},{6.83,-77.1},{12.5,-70.6},{18.1,-60.6},{
              24.5,-45.7},{32.6,-23},{50.3,31.3},{57.5,50.7},{63.9,64.6},{69.5,
              73.4},{75.2,78.6},{80,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{78,-6},{98,-26}},
              lineColor={95,95,95},
              textString="u"),Line(
              points={{-80,-80},{18,-80}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = cos(u), with -&infin; &lt; u &lt; &infin;:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/cos.png\">
</p>
</html>"));
  end cos;

  function tan "Tangent (u shall not be -pi/2, pi/2, 3*pi/2, ...)"
    extends Modelica.Math.Icons.AxisCenter;
    input SI.Angle u;
    output Real y;

  external "builtin" y = tan(u);
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
          Line(points={{-80,-80},{-78.4,-68.4},{-76.8,-59.7},{-74.4,-50},{-71.2,-40.9},
                {-67.1,-33},{-60.7,-24.8},{-51.1,-17.2},{-35.8,-9.98},{-4.42,-1.07},
                {33.4,9.12},{49.4,16.2},{59.1,23.2},{65.5,30.6},{70.4,39.1},{73.6,
                47.4},{76,56.1},{77.6,63.8},{80,80}}),
          Text(
            extent={{-90,72},{-18,24}},
            lineColor={192,192,192},
            textString="tan")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Text(
              extent={{-37,-72},{-17,-88}},
              textString="-5.8",
              lineColor={0,0,255}),Text(
              extent={{-33,86},{-13,70}},
              textString=" 5.8",
              lineColor={0,0,255}),Text(
              extent={{68,-13},{88,-33}},
              textString="1.4",
              lineColor={0,0,255}),Line(points={{-100,0},{84,0}}, color={95,95,95}),
            Polygon(
              points={{98,0},{82,6},{82,-6},{98,0}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-80,-80},{-78.4,-68.4},{-76.8,-59.7},{-74.4,-50},{-71.2,-40.9},
              {-67.1,-33},{-60.7,-24.8},{-51.1,-17.2},{-35.8,-9.98},{-4.42,-1.07},
              {33.4,9.12},{49.4,16.2},{59.1,23.2},{65.5,30.6},{70.4,39.1},{73.6,
              47.4},{76,56.1},{77.6,63.8},{80,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{82,22},{102,2}},
              lineColor={95,95,95},
              textString="u"),Line(
              points={{0,80},{86,80}},
              color={175,175,175}),Line(
              points={{80,88},{80,-16}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = tan(u), with -&infin; &lt; u &lt; &infin;
(if u is a multiple of (2n-1)*pi/2, y = tan(u) is +/- infinity).
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/tan.png\">
</p>
</html>"));
  end tan;

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

  function cosh "Hyperbolic cosine"
    extends Modelica.Math.Icons.AxisCenter;
    input Real u;
    output Real y;

  external "builtin" y = cosh(u);
    annotation (
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-90,-86.083},{68,-86.083}}, color={192,192,192}),
          Polygon(
            points={{90,-86.083},{68,-78.083},{68,-94.083},{90,-86.083}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,80},{-77.6,61.1},{-74.4,39.3},{-71.2,20.7},{-67.1,
                1.29},{-63.1,-14.6},{-58.3,-29.8},{-52.7,-43.5},{-46.2,-55.1},{-39,
                -64.3},{-30.2,-71.7},{-18.9,-77.1},{-4.42,-79.9},{10.9,-79.1},{
                23.7,-75.2},{34.2,-68.7},{42.2,-60.6},{48.6,-51.2},{54.3,-40},{
                59.1,-27.5},{63.1,-14.6},{67.1,1.29},{71.2,20.7},{74.4,39.3},{
                77.6,61.1},{80,80}}),
          Text(
            extent={{4,66},{66,20}},
            lineColor={192,192,192},
            textString="cosh")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Line(points={{-100,-84.083},{84,-84.083}}, color=
             {95,95,95}),Polygon(
              points={{98,-84.083},{82,-78.083},{82,-90.083},{98,-84.083}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-80,80},{-77.6,61.1},{-74.4,39.3},{-71.2,20.7},{-67.1,1.29},
              {-63.1,-14.6},{-58.3,-29.8},{-52.7,-43.5},{-46.2,-55.1},{-39,-64.3},
              {-30.2,-71.7},{-18.9,-77.1},{-4.42,-79.9},{10.9,-79.1},{23.7,-75.2},
              {34.2,-68.7},{42.2,-60.6},{48.6,-51.2},{54.3,-40},{59.1,-27.5},{
              63.1,-14.6},{67.1,1.29},{71.2,20.7},{74.4,39.3},{77.6,61.1},{80,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{-31,72},{-11,88}},
              textString="27",
              lineColor={0,0,255}),Text(
              extent={{64,-83},{84,-103}},
              textString="4",
              lineColor={0,0,255}),Text(
              extent={{-94,-63},{-74,-83}},
              textString="-4",
              lineColor={0,0,255}),Text(
              extent={{80,-60},{100,-80}},
              lineColor={95,95,95},
              textString="u"),Line(
              points={{0,80},{88,80}},
              color={175,175,175}),Line(
              points={{80,84},{80,-90}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = cosh(u), with -&infin; &lt; u &lt; &infin;:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/cosh.png\">
</p>
</html>"));
  end cosh;

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
          extent={{-100,-100},{100,100}}), graphics={Line(points={{-100,0},{84,0}}, color={95,95,95}),
            Polygon(
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

  function exp "Exponential, base e"
    extends Modelica.Math.Icons.AxisCenter;
    input Real u;
    output Real y;

  external "builtin" y = exp(u);
    annotation (
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-90,-80.3976},{68,-80.3976}}, color={192,192,192}),
          Polygon(
            points={{90,-80.3976},{68,-72.3976},{68,-88.3976},{90,-80.3976}},
            lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,-80},{-31,-77.9},{-6.03,-74},{10.9,-68.4},{23.7,-61},
                {34.2,-51.6},{43,-40.3},{50.3,-27.8},{56.7,-13.5},{62.3,2.23},{
                67.1,18.6},{72,38.2},{76,57.6},{80,80}}),
          Text(
            extent={{-86,50},{-14,2}},
            lineColor={192,192,192},
            textString="exp")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Line(points={{-100,-80.3976},{84,-80.3976}},
            color={95,95,95}),Polygon(
              points={{98,-80.3976},{82,-74.3976},{82,-86.3976},{98,-80.3976}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-80,-80},{-31,-77.9},{-6.03,-74},{10.9,-68.4},{23.7,-61},{
              34.2,-51.6},{43,-40.3},{50.3,-27.8},{56.7,-13.5},{62.3,2.23},{67.1,
              18.6},{72,38.2},{76,57.6},{80,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{-31,72},{-11,88}},
              textString="20",
              lineColor={0,0,255}),Text(
              extent={{-92,-81},{-72,-101}},
              textString="-3",
              lineColor={0,0,255}),Text(
              extent={{66,-81},{86,-101}},
              textString="3",
              lineColor={0,0,255}),Text(
              extent={{2,-69},{22,-89}},
              textString="1",
              lineColor={0,0,255}),Text(
              extent={{78,-54},{98,-74}},
              lineColor={95,95,95},
              textString="u"),Line(
              points={{0,80},{88,80}},
              color={175,175,175}),Line(
              points={{80,84},{80,-84}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = exp(u), with -&infin; &lt; u &lt; &infin;:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/exp.png\">
</p>
</html>"));
  end exp;

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

  function log10 "Base 10 logarithm (u shall be > 0)"
    extends Modelica.Math.Icons.AxisLeft;
    input Real u;
    output Real y;

  external "builtin" y = log10(u);
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
          Line(points={{-79.8,-80},{-79.2,-50.6},{-78.4,-37},{-77.6,-28},{-76.8,-21.3},
                {-75.2,-11.4},{-72.8,-1.31},{-69.5,8.08},{-64.7,17.9},{-57.5,28},
                {-47,38.1},{-31.8,48.1},{-10.1,58},{22.1,68},{68.7,78.1},{80,80}}),
          Text(
            extent={{-30,-22},{60,-70}},
            lineColor={192,192,192},
            textString="log10")}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Line(points={{-100,0},{84,0}}, color={95,95,95}),
            Polygon(
              points={{98,0},{82,6},{82,-6},{98,0}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),Line(
              points={{-77.8,-80},{-77.2,-50.6},{-76.4,-37},{-75.6,-28},{-74.8,-21.3},
              {-73.2,-11.4},{-70.8,-1.31},{-67.5,8.08},{-62.7,17.9},{-55.5,28},{-45,
              38.1},{-29.8,48.1},{-8.1,58},{24.1,68},{70.7,78.1},{82,80}},
              color={0,0,255},
              thickness=0.5),Text(
              extent={{66,-13},{86,-33}},
              textString="20",
              lineColor={0,0,255}),Text(
              extent={{-78,-1},{-58,-21}},
              textString="1",
              lineColor={0,0,255}),Text(
              extent={{-83,62},{-63,78}},
              textString=" 1.3",
              lineColor={0,0,255}),Text(
              extent={{80,24},{100,4}},
              lineColor={95,95,95},
              textString="u"),Text(
              extent={{-100,9},{-80,-11}},
              textString="0",
              lineColor={0,0,255}),Line(
              points={{-80,80},{86,80}},
              color={175,175,175}),Line(
              points={{80,92},{80,-12}},
              color={175,175,175})}),
      Documentation(info="<html>
<p>
This function returns y = log10(u),
with u &gt; 0:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/log10.png\">
</p>
</html>"));
  end log10;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={Line(points={{-80,0},{-68.7,34.2},{-61.5,53.1},
              {-55.1,66.4},{-49.4,74.6},{-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{
              -26.9,69.7},{-21.3,59.4},{-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,
              -50.2},{23.7,-64.2},{29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},
              {51.9,-71.5},{57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}}, color={
              0,0,0}, smooth=Smooth.Bezier)}), Documentation(info="<html>
<p>
This package contains <strong>basic mathematical functions</strong> (such as sin(..)),
as well as functions operating on
<a href=\"modelica://Modelica.Math.Vectors\">vectors</a>,
<a href=\"modelica://Modelica.Math.Matrices\">matrices</a>,
<a href=\"modelica://Modelica.Math.Nonlinear\">nonlinear functions</a>, and
<a href=\"modelica://Modelica.Math.BooleanVectors\">Boolean vectors</a>.
</p>

<h4>Main Authors</h4>
<p><a href=\"http://www.robotic.dlr.de/Martin.Otter/\"><strong>Martin Otter</strong></a>
and <strong>Marcus Baur</strong><br>
Deutsches Zentrum f&uuml;r Luft- und Raumfahrt e.V. (DLR)<br>
Institut f&uuml;r Systemdynamik und Regelungstechnik (DLR-SR)<br>
Forschungszentrum Oberpfaffenhofen<br>
D-82234 Wessling<br>
Germany<br>
email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a>
</p>

<p>
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>",   revisions="<html>
<ul>
<li><em>August 24, 2016</em>
       by Christian Kral: added wrapAngle</li>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Function tempInterpol2 added.</li>
<li><em>Oct. 24, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Icons for icon and diagram level introduced.</li>
<li><em>June 30, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized.</li>
</ul>

</html>"));
  end Math;

  package Utilities "Library of utility functions dedicated to scripting (operating on files, streams, strings, system)"
    extends Modelica.Icons.UtilitiesPackage;

    package Streams "Read from files and write to files"
      extends Modelica.Icons.FunctionsPackage;

      function error "Print error message and cancel all actions - in case of an unrecoverable error"
        extends Modelica.Icons.Function;
        input String string "String to be printed to error message window";
        external "C" ModelicaError(string) annotation(Library="ModelicaExternalC");
        annotation (Documentation(info="<html>
<h4>Syntax</h4>
<blockquote><pre>
Streams.<strong>error</strong>(string);
</pre></blockquote>
<h4>Description</h4>
<p>
In case of an unrecoverable error (i.e., if the solver is unable to recover from the error),
print the string \"string\" as error message and cancel all actions.
This function is semantically equivalent with the built-in function <strong>assert</strong> if called with the (default) <strong>AssertionLevel.error</strong>.
Line breaks are characterized by \"\\n\" in the string.
</p>
<h4>Example</h4>
<blockquote><pre>
  Streams.error(\"x (= \" + String(x) + \")\\nhas to be in the range 0 .. 1\");
</pre></blockquote>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Utilities.Streams\">Streams</a>,
<a href=\"modelica://Modelica.Utilities.Streams.print\">Streams.print</a>,
<a href=\"modelica://ModelicaReference.Operators.'assert()'\">ModelicaReference.Operators.'assert()'</a>
<a href=\"modelica://ModelicaReference.Operators.'String()'\">ModelicaReference.Operators.'String()'</a>
</p>
</html>"));
      end error;
      annotation (
        Documentation(info="<html>
<h4>Library content</h4>
<p>
Package <strong>Streams</strong> contains functions to input and output strings
to a message window or on files, as well as reading matrices from file
and writing matrices to file. Note that a string is interpreted
and displayed as html text (e.g., with print(..) or error(..))
if it is enclosed with the Modelica html quotation, e.g.,
</p>
<blockquote><p>
string = \"&lt;html&gt; first line &lt;br&gt; second line &lt;/html&gt;\".
</p></blockquote>
<p>
It is a quality of implementation, whether (a) all tags of html are supported
or only a subset, (b) how html tags are interpreted if the output device
does not allow to display formatted text.
</p>
<p>
In the table below an example call to every function is given:
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><strong><em>Function/type</em></strong></th><th><strong><em>Description</em></strong></th></tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Streams.print\">print</a>(string)<br>
          <a href=\"modelica://Modelica.Utilities.Streams.print\">print</a>(string,fileName)</td>
      <td> Print string \"string\" or vector of strings to message window or on
           file \"fileName\".</td>
  </tr>
  <tr><td>stringVector =
         <a href=\"modelica://Modelica.Utilities.Streams.readFile\">readFile</a>(fileName)</td>
      <td> Read complete text file and return it as a vector of strings.</td>
  </tr>
  <tr><td>(string, endOfFile) =
         <a href=\"modelica://Modelica.Utilities.Streams.readLine\">readLine</a>(fileName, lineNumber)</td>
      <td>Returns from the file the content of line lineNumber.</td>
  </tr>
  <tr><td>lines =
         <a href=\"modelica://Modelica.Utilities.Streams.countLines\">countLines</a>(fileName)</td>
      <td>Returns the number of lines in a file.</td>
  </tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Streams.error\">error</a>(string)</td>
      <td> Print error message \"string\" to message window
           and cancel all actions</td>
  </tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Streams.close\">close</a>(fileName)</td>
      <td> Close file if it is still open. Ignore call if
           file is already closed or does not exist. </td>
  </tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Streams.readMatrixSize\">readMatrixSize</a>(fileName, matrixName)</td>
      <td> Read dimensions of a Real matrix from a MATLAB MAT file. </td></tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Streams.readRealMatrix\">readRealMatrix</a>(fileName, matrixName, nrow, ncol)</td>
      <td> Read a Real matrix from a MATLAB MAT file. </td></tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Streams.writeRealMatrix\">writeRealMatrix</a>(fileName, matrixName, matrix, append, format)</td>
      <td> Write Real matrix to a MATLAB MAT file. </td></tr>
</table>
<p>
Use functions <strong>scanXXX</strong> from package
<a href=\"modelica://Modelica.Utilities.Strings\">Strings</a>
to parse a string.
</p>
<p>
If Real, Integer or Boolean values shall be printed
or used in an error message, they have to be first converted
to strings with the builtin operator
<a href=\"modelica://ModelicaReference.Operators.'String()'\">ModelicaReference.Operators.'String()'</a>(...).
Example:
</p>
<pre>
  <strong>if</strong> x &lt; 0 <strong>or</strong> x &gt; 1 <strong>then</strong>
     Streams.error(\"x (= \" + String(x) + \") has to be in the range 0 .. 1\");
  <strong>end if</strong>;
</pre>
</html>"));
    end Streams;

    package Strings "Operations on strings"
      extends Modelica.Icons.FunctionsPackage;

      function compare "Compare two strings lexicographically"
        extends Modelica.Icons.Function;
        input String string1;
        input String string2;
        input Boolean caseSensitive=true "= false, if case of letters is ignored";
        output Modelica.Utilities.Types.Compare result "Result of comparison";
      external "C" result = ModelicaStrings_compare(string1, string2, caseSensitive) annotation(Library="ModelicaExternalC");
        annotation (Documentation(info="<html>
<h4>Syntax</h4>
<blockquote><pre>
result = Strings.<strong>compare</strong>(string1, string2);
result = Strings.<strong>compare</strong>(string1, string2, caseSensitive=true);
</pre></blockquote>
<h4>Description</h4>
<p>
Compares two strings. If the optional argument caseSensitive=false,
upper case letters are treated as if they would be lower case letters.
The result of the comparison is returned as:
</p>
<pre>
  result = Modelica.Utilities.Types.Compare.Less     // string1 &lt; string2
         = Modelica.Utilities.Types.Compare.Equal    // string1 = string2
         = Modelica.Utilities.Types.Compare.Greater  // string1 &gt; string2
</pre>
<p>
Comparison is with regards to lexicographical order,
e.g., \"a\" &lt; \"b\";
</p>
</html>"));
      end compare;

      function isEqual "Determine whether two strings are identical"
        extends Modelica.Icons.Function;
        input String string1;
        input String string2;
        input Boolean caseSensitive=true
          "= false, if lower and upper case are ignored for the comparison";
        output Boolean identical "True, if string1 is identical to string2";
      algorithm
        identical :=compare(string1, string2, caseSensitive) == Types.Compare.Equal;
        annotation (
      Documentation(info="<html>
<h4>Syntax</h4>
<blockquote><pre>
Strings.<strong>isEqual</strong>(string1, string2);
Strings.<strong>isEqual</strong>(string1, string2, caseSensitive=true);
</pre></blockquote>
<h4>Description</h4>
<p>
Compare whether two strings are identical,
optionally ignoring case.
</p>
</html>"));
      end isEqual;
      annotation (
        Documentation(info="<html>
<h4>Library content</h4>
<p>
Package <strong>Strings</strong> contains functions to manipulate strings.
</p>
<p>
In the table below an example
call to every function is given using the <strong>default</strong> options.
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><strong><em>Function</em></strong></th><th><strong><em>Description</em></strong></th></tr>
  <tr><td>len = <a href=\"modelica://Modelica.Utilities.Strings.length\">length</a>(string)</td>
      <td>Returns length of string</td></tr>
  <tr><td>string2 = <a href=\"modelica://Modelica.Utilities.Strings.substring\">substring</a>(string1,startIndex,endIndex)
       </td>
      <td>Returns a substring defined by start and end index</td></tr>
  <tr><td>result = <a href=\"modelica://Modelica.Utilities.Strings.repeat\">repeat</a>(n)<br>
 result = <a href=\"modelica://Modelica.Utilities.Strings.repeat\">repeat</a>(n,string)</td>
      <td>Repeat a blank or a string n times.</td></tr>
  <tr><td>result = <a href=\"modelica://Modelica.Utilities.Strings.compare\">compare</a>(string1, string2)</td>
      <td>Compares two substrings with regards to alphabetical order</td></tr>
  <tr><td>identical =
<a href=\"modelica://Modelica.Utilities.Strings.isEqual\">isEqual</a>(string1,string2)</td>
      <td>Determine whether two strings are identical</td></tr>
  <tr><td>result = <a href=\"modelica://Modelica.Utilities.Strings.count\">count</a>(string,searchString)</td>
      <td>Count the number of occurrences of a string</td></tr>
  <tr>
<td>index = <a href=\"modelica://Modelica.Utilities.Strings.find\">find</a>(string,searchString)</td>
      <td>Find first occurrence of a string in another string</td></tr>
<tr>
<td>index = <a href=\"modelica://Modelica.Utilities.Strings.findLast\">findLast</a>(string,searchString)</td>
      <td>Find last occurrence of a string in another string</td></tr>
  <tr><td>string2 = <a href=\"modelica://Modelica.Utilities.Strings.replace\">replace</a>(string,searchString,replaceString)</td>
      <td>Replace one or all occurrences of a string</td></tr>
  <tr><td>stringVector2 = <a href=\"modelica://Modelica.Utilities.Strings.sort\">sort</a>(stringVector1)</td>
      <td>Sort vector of strings in alphabetic order</td></tr>
  <tr><td>hash = <a href=\"modelica://Modelica.Utilities.Strings.hashString\">hashString</a>(string)</td>
      <td>Create a hash value of a string</td></tr>
  <tr><td>(token, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanToken\">scanToken</a>(string,startIndex)</td>
      <td>Scan for a token (Real/Integer/Boolean/String/Identifier/Delimiter/NoToken)</td></tr>
  <tr><td>(number, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanReal\">scanReal</a>(string,startIndex)</td>
      <td>Scan for a Real constant</td></tr>
  <tr><td>(number, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanInteger\">scanInteger</a>(string,startIndex)</td>
      <td>Scan for an Integer constant</td></tr>
  <tr><td>(boolean, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanBoolean\">scanBoolean</a>(string,startIndex)</td>
      <td>Scan for a Boolean constant</td></tr>
  <tr><td>(string2, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanString\">scanString</a>(string,startIndex)</td>
      <td>Scan for a String constant</td></tr>
  <tr><td>(identifier, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanIdentifier\">scanIdentifier</a>(string,startIndex)</td>
      <td>Scan for an identifier</td></tr>
  <tr><td>(delimiter, index) = <a href=\"modelica://Modelica.Utilities.Strings.scanDelimiter\">scanDelimiter</a>(string,startIndex)</td>
      <td>Scan for delimiters</td></tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Strings.scanNoToken\">scanNoToken</a>(string,startIndex)</td>
      <td>Check that remaining part of string consists solely of<br>
          white space or line comments (\"// ...\\n\").</td></tr>
  <tr><td><a href=\"modelica://Modelica.Utilities.Strings.syntaxError\">syntaxError</a>(string,index,message)</td>
      <td> Print a \"syntax error message\" as well as a string and the<br>
           index at which scanning detected an error</td></tr>
</table>
<p>
The functions \"compare\", \"isEqual\", \"count\", \"find\", \"findLast\", \"replace\", \"sort\"
have the optional
input argument <strong>caseSensitive</strong> with default <strong>true</strong>.
If <strong>false</strong>, the operation is carried out without taking
into account whether a character is upper or lower case.
</p>
</html>"));
    end Strings;

    package Types "Type definitions used in package Modelica.Utilities"
      extends Modelica.Icons.TypesPackage;

      type Compare = enumeration(
          Less "String 1 is lexicographically less than string 2",
          Equal "String 1 is identical to string 2",
          Greater "String 1 is lexicographically greater than string 2")
        "Enumeration defining comparison of two strings";
      annotation (Documentation(info="<html>
<p>
This package contains type definitions used in Modelica.Utilities.
</p>

</html>"));
    end Types;
      annotation (
  Documentation(info="<html>
<p>
This package contains Modelica <strong>functions</strong> that are
especially suited for <strong>scripting</strong>. The functions might
be used to work with strings, read data from file, write data
to file or copy, move and remove files.
</p>
<p>
For an introduction, have especially a look at:
</p>
<ul>
<li> <a href=\"modelica://Modelica.Utilities.UsersGuide\">Modelica.Utilities.User's Guide</a>
     discusses the most important aspects of this library.</li>
<li> <a href=\"modelica://Modelica.Utilities.Examples\">Modelica.Utilities.Examples</a>
     contains examples that demonstrate the usage of this library.</li>
</ul>
<p>
The following main sublibraries are available:
</p>
<ul>
<li> <a href=\"modelica://Modelica.Utilities.Files\">Files</a>
     provides functions to operate on files and directories, e.g.,
     to copy, move, remove files.</li>
<li> <a href=\"modelica://Modelica.Utilities.Streams\">Streams</a>
     provides functions to read from files and write to files.</li>
<li> <a href=\"modelica://Modelica.Utilities.Strings\">Strings</a>
     provides functions to operate on strings. E.g.
     substring, find, replace, sort, scanToken.</li>
<li> <a href=\"modelica://Modelica.Utilities.System\">System</a>
     provides functions to interact with the environment.
     E.g., get or set the working directory or environment
     variables and to send a command to the default shell.</li>
</ul>

<p>
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>"));
  end Utilities;

  package Constants "Library of mathematical constants and constants of nature (e.g., pi, eps, R, sigma)"
    import SI = Modelica.SIunits;
    import NonSI = Modelica.SIunits.Conversions.NonSIunits;
    extends Modelica.Icons.Package;

    final constant Real pi=2*Modelica.Math.asin(1.0);

    final constant Real eps=ModelicaServices.Machine.eps
      "Biggest number such that 1.0 + eps = 1.0";

    final constant Real small=ModelicaServices.Machine.small
      "Smallest number such that small and -small are representable on the machine";

    final constant Real inf=ModelicaServices.Machine.inf
      "Biggest Real number such that inf and -inf are representable on the machine";

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
<dd><strong>CODATA Recommended Values of the Fundamental Physical Constants: 2014</strong>.
<a href= \"http://dx.doi.org/10.5281/zenodo.22826\">http://dx.doi.org/10.5281/zenodo.22826</a>, 2015. See also <a href=
\"http://physics.nist.gov/cuu/Constants/index.html\">http://physics.nist.gov/cuu/Constants/index.html</a></dd>
</dl>

<p>CODATA is the Committee on Data for Science and Technology.</p>

<dl>
<dt><strong>Main Author:</strong></dt>
<dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a><br>
    Deutsches Zentrum f&uuml;r Luft und Raumfahrt e. V. (DLR)<br>
    Oberpfaffenhofen<br>
    Postfach 1116<br>
    D-82230 We&szlig;ling<br>
    email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a></dd>
</dl>

<p>
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>",   revisions="<html>
<ul>
<li><em>Nov 4, 2015</em>
       by Thomas Beutlich:<br>
       Constants updated according to 2014 CODATA values.</li>
<li><em>Nov 8, 2004</em>
       by Christian Schweiger:<br>
       Constants updated according to 2002 CODATA values.</li>
<li><em>Dec 9, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Constants updated according to 1998 CODATA values. Using names, values
       and description text from this source. Included magnetic and
       electric constant.</li>
<li><em>Sep 18, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Constants eps, inf, small introduced.</li>
<li><em>Nov 15, 1997</em>
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
              radius=25.0)}), Documentation(info="<html>
<p>Standard package icon.</p>
</html>"));
    end Package;

    partial package BasesPackage "Icon for packages containing base classes"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(
              extent={{-30.0,-30.0},{30.0,30.0}},
              lineColor={128,128,128},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
                                Documentation(info="<html>
<p>This icon shall be used for a package/library that contains base models and classes, respectively.</p>
</html>"));
    end BasesPackage;

    partial package VariantsPackage "Icon for package containing variants"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},
                {100,100}}), graphics={
            Ellipse(
              origin={10.0,10.0},
              fillColor={76,76,76},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-80.0,-80.0},{-20.0,-20.0}}),
            Ellipse(
              origin={10.0,10.0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{0.0,-80.0},{60.0,-20.0}}),
            Ellipse(
              origin={10.0,10.0},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{0.0,0.0},{60.0,60.0}}),
            Ellipse(
              origin={10.0,10.0},
              lineColor={128,128,128},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              extent={{-80.0,0.0},{-20.0,60.0}})}),
                                Documentation(info="<html>
<p>This icon shall be used for a package/library that contains several variants of one component.</p>
</html>"));
    end VariantsPackage;

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

    partial package SensorsPackage "Icon for packages containing sensors"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(origin={0.0,-30.0},
              fillColor={255,255,255},
              extent={{-90.0,-90.0},{90.0,90.0}},
              startAngle=20.0,
              endAngle=160.0),
            Ellipse(origin={0.0,-30.0},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-20.0,-20.0},{20.0,20.0}}),
            Line(origin={0.0,-30.0},
              points={{0.0,60.0},{0.0,90.0}}),
            Ellipse(origin={-0.0,-30.0},
              fillColor={64,64,64},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-10.0,-10.0},{10.0,10.0}}),
            Polygon(
              origin={-0.0,-30.0},
              rotation=-35.0,
              fillColor={64,64,64},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-7.0,0.0},{-3.0,85.0},{0.0,90.0},{3.0,85.0},{7.0,0.0}})}),
                                Documentation(info="<html>
<p>This icon indicates a package containing sensors.</p>
</html>"));
    end SensorsPackage;

    partial package UtilitiesPackage "Icon for utility packages"
      extends Modelica.Icons.Package;
       annotation (Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
      Polygon(
        origin={1.3835,-4.1418},
        rotation=45.0,
        fillColor={64,64,64},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{-15.0,93.333},{-15.0,68.333},{0.0,58.333},{15.0,68.333},{15.0,93.333},{20.0,93.333},{25.0,83.333},{25.0,58.333},{10.0,43.333},{10.0,-41.667},{25.0,-56.667},{25.0,-76.667},{10.0,-91.667},{0.0,-91.667},{0.0,-81.667},{5.0,-81.667},{15.0,-71.667},{15.0,-61.667},{5.0,-51.667},{-5.0,-51.667},{-15.0,-61.667},{-15.0,-71.667},{-5.0,-81.667},{0.0,-81.667},{0.0,-91.667},{-10.0,-91.667},{-25.0,-76.667},{-25.0,-56.667},{-10.0,-41.667},{-10.0,43.333},{-25.0,58.333},{-25.0,83.333},{-20.0,93.333}}),
      Polygon(
        origin={10.1018,5.218},
        rotation=-45.0,
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid,
        points={{-15.0,87.273},{15.0,87.273},{20.0,82.273},{20.0,27.273},{10.0,17.273},{10.0,7.273},{20.0,2.273},{20.0,-2.727},{5.0,-2.727},{5.0,-77.727},{10.0,-87.727},{5.0,-112.727},{-5.0,-112.727},{-10.0,-87.727},{-5.0,-77.727},{-5.0,-2.727},{-20.0,-2.727},{-20.0,2.273},{-10.0,7.273},{-10.0,17.273},{-20.0,27.273},{-20.0,82.273}})}),
      Documentation(info="<html>
<p>This icon indicates a package containing utility classes.</p>
</html>"));
    end UtilitiesPackage;

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
              smooth=Smooth.Bezier), Polygon(
              origin={2.7403,1.6673},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{49.2597,22.3327},{31.2597,24.3327},{7.2597,18.3327},{-26.7403,
                10.3327},{-46.7403,14.3327},{-48.7403,6.3327},{-32.7403,0.3327},{-6.7403,
                4.3327},{33.2597,14.3327},{49.2597,14.3327},{49.2597,22.3327}},
              smooth=Smooth.Bezier)}));
    end TypesPackage;

    partial package FunctionsPackage "Icon for packages containing functions"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
              Text(
                lineColor={128,128,128},
                extent={{-90,-90},{90,90}},
                textString="f")}));
    end FunctionsPackage;

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
              smooth=Smooth.Bezier), Ellipse(
              origin={-0.5,56.5},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-12.5,-12.5},{12.5,12.5}})}));
    end IconsPackage;

    partial package InternalPackage "Icon for an internal package (indicating that the package should not be directly utilized by user)"
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={
          Rectangle(
            lineColor={215,215,215},
            fillColor={255,255,255},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25),
          Rectangle(
            lineColor={215,215,215},
            extent={{-100,-100},{100,100}},
            radius=25),
          Ellipse(
            extent={{-80,80},{80,-80}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-55,55},{55,-55}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-60,14},{60,-14}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            rotation=45)}),
      Documentation(info="<html>

<p>
This icon shall be used for a package that contains internal classes not to be
directly utilized by a user.
</p>
</html>"));
    end InternalPackage;

    partial package MaterialPropertiesPackage "Icon for package containing property classes"
      extends Modelica.Icons.Package;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(
              lineColor={102,102,102},
              fillColor={204,204,204},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Sphere,
              extent={{-60.0,-60.0},{60.0,60.0}})}),
                                Documentation(info="<html>
<p>This icon indicates a package that contains properties</p>
</html>"));
    end MaterialPropertiesPackage;

    partial class RotationalSensor "Icon representing a round measurement device"

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              fillColor={245,245,245},
              fillPattern=FillPattern.Solid,
              extent={{-70.0,-70.0},{70.0,70.0}}),
            Line(points={{0.0,70.0},{0.0,40.0}}),
            Line(points={{22.9,32.8},{40.2,57.3}}),
            Line(points={{-22.9,32.8},{-40.2,57.3}}),
            Line(points={{37.6,13.7},{65.8,23.9}}),
            Line(points={{-37.6,13.7},{-65.8,23.9}}),
            Ellipse(
              lineColor={64,64,64},
              fillColor={255,255,255},
              extent={{-12.0,-12.0},{12.0,12.0}}),
            Polygon(
              rotation=-17.5,
              fillColor={64,64,64},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-5.0,0.0},{-2.0,60.0},{0.0,65.0},{2.0,60.0},{5.0,0.0}}),
            Ellipse(
              fillColor={64,64,64},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-7.0,-7.0},{7.0,7.0}})}),
        Documentation(info="<html>
<p>
This icon is designed for a <strong>rotational sensor</strong> model.
</p>
</html>"));
    end RotationalSensor;

    partial class TranslationalSensor
      "Icon representing a linear measurement device"

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              fillColor={245,245,245},
              fillPattern=FillPattern.Solid,
              extent={{-70.0,-60.0},{70.0,20.0}}),
            Polygon(
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{0.0,-40.0},{-10.0,-16.0},{10.0,-16.0},{0.0,-40.0}}),
            Line(points={{0.0,0.0},{0.0,-16.0}}),
            Line(points={{-70.0,0.0},{0.0,0.0}}),
            Line(points={{-50.0,-40.0},{-50.0,-60.0}}),
            Line(points={{-30.0,-40.0},{-30.0,-60.0}}),
            Line(points={{-10.0,-40.0},{-10.0,-60.0}}),
            Line(points={{10.0,-40.0},{10.0,-60.0}}),
            Line(points={{30.0,-40.0},{30.0,-60.0}}),
            Line(points={{50.0,-40.0},{50.0,-60.0}})}),
        Documentation(info="<html>
<p>
This icon is designed for a <strong>translational sensor</strong> model.
</p></html>"));
    end TranslationalSensor;

    partial function Function "Icon for functions"

      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Text(
              lineColor={0,0,255},
              extent={{-150,105},{150,145}},
              textString="%name"),
            Ellipse(
              lineColor = {108,88,49},
              fillColor = {255,215,136},
              fillPattern = FillPattern.Solid,
              extent = {{-100,-100},{100,100}}),
            Text(
              lineColor={108,88,49},
              extent={{-90.0,-90.0},{90.0,90.0}},
              textString="f")}),
    Documentation(info="<html>
<p>This icon indicates Modelica functions.</p>
</html>"));
    end Function;

    partial record Record "Icon for records"

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
            Text(
              lineColor={0,0,255},
              extent={{-150,60},{150,100}},
              textString="%name"),
            Rectangle(
              origin={0.0,-25.0},
              lineColor={64,64,64},
              fillColor={255,215,136},
              fillPattern=FillPattern.Solid,
              extent={{-100.0,-75.0},{100.0,75.0}},
              radius=25.0),
            Line(
              points={{-100.0,0.0},{100.0,0.0}},
              color={64,64,64}),
            Line(
              origin={0.0,-50.0},
              points={{-100.0,0.0},{100.0,0.0}},
              color={64,64,64}),
            Line(
              origin={0.0,-25.0},
              points={{0.0,75.0},{0.0,-75.0}},
              color={64,64,64})}), Documentation(info="<html>
<p>
This icon is indicates a record.
</p>
</html>"));
    end Record;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Polygon(
              origin={-8.167,-17},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-15.833,20.0},{-15.833,30.0},{14.167,40.0},{24.167,20.0},{
                  4.167,-30.0},{14.167,-30.0},{24.167,-30.0},{24.167,-40.0},{-5.833,
                  -50.0},{-15.833,-30.0},{4.167,20.0},{-5.833,20.0}},
              smooth=Smooth.Bezier), Ellipse(
              origin={-0.5,56.5},
              fillColor={128,128,128},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-12.5,-12.5},{12.5,12.5}})}), Documentation(info="<html>
<p>This package contains definitions for the graphical layout of components which may be used in different libraries. The icons can be utilized by inheriting them in the desired class using &quot;extends&quot; or by directly copying the &quot;icon&quot; layer.</p>

<h4>Main Authors:</h4>

<dl>
<dt><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a></dt>
    <dd>Deutsches Zentrum fuer Luft und Raumfahrt e.V. (DLR)</dd>
    <dd>Oberpfaffenhofen</dd>
    <dd>Postfach 1116</dd>
    <dd>D-82230 Wessling</dd>
    <dd>email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a></dd>
<dt>Christian Kral</dt>

    <dd>  <a href=\"https://christiankral.net/\">Electric Machines, Drives and Systems</a><br>
</dd>
    <dd>1060 Vienna, Austria</dd>
    <dd>email: <a href=\"mailto:dr.christian.kral@gmail.com\">dr.christian.kral@gmail.com</a></dd>
<dt>Johan Andreasson</dt>
    <dd><a href=\"http://www.modelon.se/\">Modelon AB</a></dd>
    <dd>Ideon Science Park</dd>
    <dd>22370 Lund, Sweden</dd>
    <dd>email: <a href=\"mailto:johan.andreasson@modelon.se\">johan.andreasson@modelon.se</a></dd>
</dl>

<p>
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>"));
  end Icons;

  package SIunits "Library of type and unit definitions based on SI units according to ISO 31-1992"
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

    package Conversions "Conversion functions to/from non SI units and type definitions of non SI units"
      extends Modelica.Icons.Package;

      package NonSIunits "Type definitions of non SI units"
        extends Modelica.Icons.Package;

        type Temperature_degC = Real (final quantity="ThermodynamicTemperature",
              final unit="degC")
          "Absolute temperature in degree Celsius (for relative temperature use SIunits.TemperatureDifference)" annotation(absoluteValue=true);

        type Pressure_bar = Real (final quantity="Pressure", final unit="bar")
          "Absolute pressure in bar";
        annotation (Documentation(info="<html>
<p>
This package provides predefined types, such as <strong>Angle_deg</strong> (angle in
degree), <strong>AngularVelocity_rpm</strong> (angular velocity in revolutions per
minute) or <strong>Temperature_degF</strong> (temperature in degree Fahrenheit),
which are in common use but are not part of the international standard on
units according to ISO 31-1992 \"General principles concerning quantities,
units and symbols\" and ISO 1000-1992 \"SI units and recommendations for
the use of their multiples and of certain other units\".</p>
<p>If possible, the types in this package should not be used. Use instead
types of package Modelica.SIunits. For more information on units, see also
the book of Francois Cardarelli <strong>Scientific Unit Conversion - A
Practical Guide to Metrication</strong> (Springer 1997).</p>
<p>Some units, such as <strong>Temperature_degC/Temp_C</strong> are both defined in
Modelica.SIunits and in Modelica.Conversions.NonSIunits. The reason is that these
definitions have been placed erroneously in Modelica.SIunits although they
are not SIunits. For backward compatibility, these type definitions are
still kept in Modelica.SIunits.</p>
</html>"),   Icon(coordinateSystem(extent={{-100,-100},{100,100}}), graphics={
        Text(
          origin={15.0,51.8518},
          extent={{-105.0,-86.8518},{75.0,-16.8518}},
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
                textString="K"), Text(
                extent={{100,-20},{20,-100}},
                textString="degC")}));
      end to_degC;

      function from_degC "Convert from degCelsius to Kelvin"
        extends Modelica.SIunits.Icons.Conversion;
        input NonSIunits.Temperature_degC Celsius "Celsius value";
        output Temperature Kelvin "Kelvin value";
      algorithm
        Kelvin := Celsius - Modelica.Constants.T_zero;
        annotation (Inline=true,Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                extent={{-20,100},{-100,20}},
                textString="degC"), Text(
                extent={{100,-20},{20,-100}},
                textString="K")}));
      end from_degC;

      function to_bar "Convert from Pascal to bar"
        extends Modelica.SIunits.Icons.Conversion;
        input Pressure Pa "Pascal value";
        output NonSIunits.Pressure_bar bar "bar value";
      algorithm
        bar := Pa/1e5;
        annotation (Inline=true,Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                extent={{-12,100},{-100,56}},
                textString="Pa"), Text(
                extent={{98,-52},{-4,-100}},
                textString="bar")}));
      end to_bar;
      annotation (Documentation(info="<html>
<p>This package provides conversion functions from the non SI Units
defined in package Modelica.SIunits.Conversions.NonSIunits to the
corresponding SI Units defined in package Modelica.SIunits and vice
versa. It is recommended to use these functions in the following
way (note, that all functions have one Real input and one Real output
argument):</p>
<pre>
  <strong>import</strong> SI = Modelica.SIunits;
  <strong>import</strong> Modelica.SIunits.Conversions.*;
     ...
  <strong>parameter</strong> SI.Temperature     T   = from_degC(25);   // convert 25 degree Celsius to Kelvin
  <strong>parameter</strong> SI.Angle           phi = from_deg(180);   // convert 180 degree to radian
  <strong>parameter</strong> SI.AngularVelocity w   = from_rpm(3600);  // convert 3600 revolutions per minutes
                                                      // to radian per seconds
</pre>

</html>"));
    end Conversions;

    type Angle = Real (
        final quantity="Angle",
        final unit="rad",
        displayUnit="deg");

    type Length = Real (final quantity="Length", final unit="m");

    type Height = Length(min=0);

    type Diameter = Length(min=0);

    type Area = Real (final quantity="Area", final unit="m2");

    type Volume = Real (final quantity="Volume", final unit="m3");

    type Time = Real (final quantity="Time", final unit="s");

    type Velocity = Real (final quantity="Velocity", final unit="m/s");

    type Acceleration = Real (final quantity="Acceleration", final unit="m/s2");

    type Frequency = Real (final quantity="Frequency", final unit="Hz");

    type Mass = Real (
        quantity="Mass",
        final unit="kg",
        min=0);

    type Density = Real (
        final quantity="Density",
        final unit="kg/m3",
        displayUnit="g/cm3",
        min=0.0);

    type Pressure = Real (
        final quantity="Pressure",
        final unit="Pa",
        displayUnit="bar");

    type AbsolutePressure = Pressure (min=0.0, nominal = 1e5);

    type PressureDifference = Pressure;

    type DynamicViscosity = Real (
        final quantity="DynamicViscosity",
        final unit="Pa.s",
        min=0);

    type Energy = Real (final quantity="Energy", final unit="J");

    type Power = Real (final quantity="Power", final unit="W");

    type EnergyFlowRate = Power;

    type EnthalpyFlowRate = Real (final quantity="EnthalpyFlowRate", final unit=
            "W");

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
      "Absolute temperature (use type TemperatureDifference for relative temperatures)" annotation(absoluteValue=true);

    type Temperature = ThermodynamicTemperature;

    type TemperatureDifference = Real (
        final quantity="ThermodynamicTemperature",
        final unit="K") annotation(absoluteValue=false);

    type Compressibility = Real (final quantity="Compressibility", final unit=
            "1/Pa");

    type IsothermalCompressibility = Compressibility;

    type HeatFlowRate = Real (final quantity="Power", final unit="W");

    type ThermalConductivity = Real (final quantity="ThermalConductivity", final unit=
               "W/(m.K)");

    type CoefficientOfHeatTransfer = Real (final quantity=
            "CoefficientOfHeatTransfer", final unit="W/(m2.K)");

    type HeatCapacity = Real (final quantity="HeatCapacity", final unit="J/K");

    type SpecificHeatCapacity = Real (final quantity="SpecificHeatCapacity",
          final unit="J/(kg.K)");

    type RatioOfSpecificHeatCapacities = Real (final quantity=
            "RatioOfSpecificHeatCapacities", final unit="1");

    type SpecificEntropy = Real (final quantity="SpecificEntropy",
                                 final unit="J/(kg.K)");

    type SpecificEnergy = Real (final quantity="SpecificEnergy",
                                final unit="J/kg");

    type SpecificEnthalpy = SpecificEnergy;

    type DerDensityByEnthalpy = Real (final unit="kg.s2/m5");

    type DerDensityByPressure = Real (final unit="s2/m2");

    type DerDensityByTemperature = Real (final unit="kg/(m3.K)");

    type MolarMass = Real (final quantity="MolarMass", final unit="kg/mol",min=0);

    type MassFraction = Real (final quantity="MassFraction", final unit="1",
                              min=0, max=1);

    type ReynoldsNumber = Real (final quantity="ReynoldsNumber", final unit="1");

    type PrandtlNumber = Real (final quantity="PrandtlNumber", final unit="1");
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={
        Polygon(
          fillColor = {128,128,128},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-80,-40},{-80,-40},{-55,50},{-52.5,62.5},{-65,60},{-65,65},{-35,77.5},{-32.5,60},{-50,0},{-50,0},{-30,15},{-20,27.5},{-32.5,27.5},{-32.5,27.5},{-32.5,32.5},{-32.5,32.5},{2.5,32.5},{2.5,32.5},{2.5,27.5},{2.5,27.5},{-7.5,27.5},{-30,7.5},{-30,7.5},{-25,-25},{-17.5,-28.75},{-10,-25},{-5,-26.25},{-5,-32.5},{-16.25,-41.25},{-31.25,-43.75},{-40,-33.75},{-45,-5},{-45,-5},{-52.5,-10},{-52.5,-10},{-60,-40},{-60,-40}},
          smooth = Smooth.Bezier),
        Polygon(
          fillColor = {128,128,128},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{87.5,30},{62.5,30},{62.5,30},{55,33.75},{36.25,35},{16.25,25},{7.5,6.25},{11.25,-7.5},{22.5,-12.5},{22.5,-12.5},{6.25,-22.5},{6.25,-35},{16.25,-38.75},{16.25,-38.75},{21.25,-41.25},{21.25,-41.25},{45,-48.75},{47.5,-61.25},{32.5,-70},{12.5,-65},{7.5,-51.25},{21.25,-41.25},{21.25,-41.25},{16.25,-38.75},{16.25,-38.75},{6.25,-41.25},{-6.25,-50},{-3.75,-68.75},{30,-76.25},{65,-62.5},{63.75,-35},{27.5,-26.25},{22.5,-20},{27.5,-15},{27.5,-15},{30,-7.5},{30,-7.5},{27.5,-2.5},{28.75,11.25},{36.25,27.5},{47.5,30},{53.75,22.5},{51.25,8.75},{45,-6.25},{35,-11.25},{30,-7.5},{30,-7.5},{27.5,-15},{27.5,-15},{43.75,-16.25},{65,-6.25},{72.5,10},{70,20},{70,20},{80,20}},
          smooth = Smooth.Bezier)}), Documentation(info="<html>
<p>This package provides predefined types, such as <em>Mass</em>,
<em>Angle</em>, <em>Time</em>, based on the international standard
on units, e.g.,
</p>

<pre>   <strong>type</strong> Angle = Real(<strong>final</strong> quantity = \"Angle\",
                     <strong>final</strong> unit     = \"rad\",
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
Copyright &copy; 1998-2019, Modelica Association and contributors
</p>
</html>",   revisions="<html>
<ul>
<li><em>May 25, 2011</em> by Stefan Wischhusen:<br/>Added molar units for energy and enthalpy.</li>
<li><em>Jan. 27, 2010</em> by Christian Kral:<br/>Added complex units.</li>
<li><em>Dec. 14, 2005</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Add User&#39;s Guide and removed &quot;min&quot; values for Resistance and Conductance.</li>
<li><em>October 21, 2002</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and Christian Schweiger:<br/>Added new package <strong>Conversions</strong>. Corrected typo <em>Wavelenght</em>.</li>
<li><em>June 6, 2000</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Introduced the following new types<br/>type Temperature = ThermodynamicTemperature;<br/>types DerDensityByEnthalpy, DerDensityByPressure, DerDensityByTemperature, DerEnthalpyByPressure, DerEnergyByDensity, DerEnergyByPressure<br/>Attribute &quot;final&quot; removed from min and max values in order that these values can still be changed to narrow the allowed range of values.<br/>Quantity=&quot;Stress&quot; removed from type &quot;Stress&quot;, in order that a type &quot;Stress&quot; can be connected to a type &quot;Pressure&quot;.</li>
<li><em>Oct. 27, 1999</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>New types due to electrical library: Transconductance, InversePotential, Damping.</li>
<li><em>Sept. 18, 1999</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Renamed from SIunit to SIunits. Subpackages expanded, i.e., the SIunits package, does no longer contain subpackages.</li>
<li><em>Aug 12, 1999</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Type &quot;Pressure&quot; renamed to &quot;AbsolutePressure&quot; and introduced a new type &quot;Pressure&quot; which does not contain a minimum of zero in order to allow convenient handling of relative pressure. Redefined BulkModulus as an alias to AbsolutePressure instead of Stress, since needed in hydraulics.</li>
<li><em>June 29, 1999</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br/>Bug-fix: Double definition of &quot;Compressibility&quot; removed and appropriate &quot;extends Heat&quot; clause introduced in package SolidStatePhysics to incorporate ThermodynamicTemperature.</li>
<li><em>April 8, 1998</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and Astrid Jaschinski:<br/>Complete ISO 31 chapters realized.</li>
<li><em>Nov. 15, 1997</em> by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and Hubertus Tummescheit:<br/>Some chapters realized.</li>
</ul>
</html>"));
  end SIunits;
annotation (
preferredView="info",
version="3.2.3",
versionBuild=2,
versionDate="2019-01-23",
dateModified = "2019-03-20 12:00:00Z",
revisionId="8f65f621a 2019-03-20 09:22:19 +0100",
uses(Complex(version="3.2.3"), ModelicaServices(version="3.2.3")),
conversion(
 noneFromVersion="3.2.2",
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
Package <strong>Modelica&reg;</strong> is a <strong>standardized</strong> and <strong>free</strong> package
that is developed together with the Modelica&reg; language from the
Modelica Association, see
<a href=\"https://www.Modelica.org\">https://www.Modelica.org</a>.
It is also called <strong>Modelica Standard Library</strong>.
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
<li> The <strong>Examples</strong> packages in the various libraries, demonstrate
  how to use the components of the corresponding sublibrary.</li>
</ul>

<p>
This version of the Modelica Standard Library consists of
</p>
<ul>
<li><strong>1288</strong> component models and blocks,</li>
<li><strong>404</strong> example models, and</li>
<li><strong>1227</strong> functions</li>
</ul>
<p>
that are directly usable (= number of public, non-partial, non-internal and non-obsolete classes). It is fully compliant
to <a href=\"https://www.modelica.org/documents/ModelicaSpec32Revision2.pdf\">Modelica Specification Version 3.2 Revision 2</a>
and it has been tested with Modelica tools from different vendors.
</p>

<p>
<strong>Licensed by the Modelica Association under the 3-Clause BSD License</strong><br>
Copyright &copy; 1998-2019, Modelica Association and <a href=\"modelica://Modelica.UsersGuide.Contact\">contributors</a>.
</p>

<p>
<em>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the 3-Clause BSD license. For license conditions (including the disclaimer of warranty) visit <a href=\"https://modelica.org/licenses/modelica-3-clause-bsd\">https://modelica.org/licenses/modelica-3-clause-bsd</a>.</em>
</p>

<p>
<strong>Modelica&reg;</strong> is a registered trademark of the Modelica Association.
</p>
</html>"));
end Modelica;

package MeFlexWaermeLib

  package Core

    package Buildings "Library with models for building energy and control systems"
      extends Modelica.Icons.Package;

      package Fluid "Package with models for fluid flow systems"
        extends Modelica.Icons.Package;

        package FixedResistances "Package with models for fixed flow resistances"
          extends Modelica.Icons.VariantsPackage;

          model HydraulicDiameter "Fixed flow resistance with hydraulic diameter and m_flow as parameter"
            extends Buildings.Fluid.FixedResistances.PressureDrop(final deltaM=eta_default*dh/4*Modelica.Constants.pi*ReC/m_flow_nominal_pos, final dp_nominal=fac*dpStraightPipe_nominal);

            parameter Modelica.SIunits.Length dh=sqrt(4*m_flow_nominal/rho_default/v_nominal/Modelica.Constants.pi)
              "Hydraulic diameter (assuming a round cross section area)";

            parameter Modelica.SIunits.Length length "Length of the pipe";

            parameter Real ReC(min=0)=4000
              "Reynolds number where transition to turbulent starts";

            parameter Modelica.SIunits.Velocity v_nominal = if rho_default < 500 then 1.5 else 0.15
              "Velocity at m_flow_nominal (used to compute default value for hydraulic diameter dh)"
              annotation(Dialog(group="Nominal condition"));

            parameter Modelica.SIunits.Length roughness(min=0) = 2.5e-5
              "Absolute roughness of pipe, with a default for a smooth steel pipe (dummy if use_roughness = false)";

            parameter Real fac(min=1) = 2
              "Factor to take into account resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal";

            final parameter Modelica.SIunits.PressureDifference dpStraightPipe_nominal(displayUnit="Pa")=
                Modelica.Fluid.Pipes.BaseClasses.WallFriction.Detailed.pressureLoss_m_flow(
                m_flow=m_flow_nominal,
                rho_a=rho_default,
                rho_b=rho_default,
                mu_a=mu_default,
                mu_b=mu_default,
                length=length,
                diameter=dh,
                roughness=roughness,
                m_flow_small=m_flow_small)
              "Pressure loss of a straight pipe at m_flow_nominal";

            Modelica.SIunits.Velocity v = m_flow/(rho_default*ARound)
              "Flow velocity (assuming a round cross section area)";

          protected
            parameter Modelica.SIunits.Area ARound = dh^2*Modelica.Constants.pi/4
               "Cross sectional area (assuming a round cross section area)";

            parameter Medium.ThermodynamicState state_default=
              Medium.setState_pTX(
                T=Medium.T_default,
                p=Medium.p_default,
                X=Medium.X_default[1:Medium.nXi]) "Default state";

            parameter Modelica.SIunits.Density rho_default = Medium.density(state_default)
              "Density at nominal condition";

            parameter Modelica.SIunits.DynamicViscosity mu_default = Medium.dynamicViscosity(
                state_default)
              "Dynamic viscosity at nominal condition";

          annotation (defaultComponentName="res",
          Documentation(info="<html>
<p>
This is a model of a flow resistance with a fixed flow coefficient.
The mass flow rate is computed as
</p>
<p align=\"center\" style=\"font-style:italic;\">
m&#775; = k
&radic;<span style=\"text-decoration:overline;\">&Delta;P</span>,
</p>
<p>
where
<i>k</i> is a constant and
<i>&Delta;P</i> is the pressure drop.
The constant <i>k</i> is equal to
<code>k=m_flow_nominal/sqrt(dp_nominal)</code>,
where <code>m_flow_nominal</code> is a parameter.
</p>
<h4>Assumptions</h4>
<p>
In the region
<code>abs(m_flow) &lt; m_flow_turbulent</code>,
the square root is replaced by a differentiable function
with finite slope.
The value of <code>m_flow_turbulent</code> is
computed as
<code>m_flow_turbulent = eta_nominal*dh/4*&pi;*ReC</code>,
where
<code>eta_nominal</code> is the dynamic viscosity, obtained from
the medium model. The parameter
<code>dh</code> is the hydraulic diameter and
<code>ReC=4000</code> is the critical Reynolds number, which both
can be set by the user.
</p>
<h4>Important parameters</h4>
<p>
By default, the pressure drop at nominal flow rate is computed as
</p>
<pre>
dp_nominal = fac * dpStraightPipe_nominal,
</pre>
<p>
where <code>dpStraightPipe_nominal</code> is a parameter that is automatically computed
based on the
nominal mass flow rate, hydraulic diameter, pipe roughness and medium properties.
The hydraulic diameter <code>dh</code> is by default
computed based on the flow velocity <code>v_nominal</code> and the nominal
mass flow rate <code>m_flow_nominal</code>. Hence, users should change the
default values of <code>dh</code> or <code>v_nominal</code>
if they are not applicable for their model.
</p>
<p>
The factor <code>fac</code> takes into account additional resistances such as
for bends. The default value of <code>2</code> can be changed by the user.
</p>
<p>
The parameter <code>from_dp</code> is used to determine
whether the mass flow rate is computed as a function of the
pressure drop (if <code>from_dp=true</code>), or vice versa.
This setting can affect the size of the nonlinear system of equations.
</p>
<p>
If the parameter <code>linearized</code> is set to <code>true</code>,
then the pressure drop is computed as a linear function of the
mass flow rate.
</p>
<p>
Setting <code>allowFlowReversal=false</code> can lead to simpler
equations. However, this should only be set to <code>false</code>
if one can guarantee that the flow never reverses its direction.
This can be difficult to guarantee, as pressure imbalance after
the initialization, or due to medium expansion and contraction,
can lead to reverse flow.
</p>
<p>
If the parameter
<code>show_T</code> is set to <code>true</code>,
then the model will compute the
temperature at its ports. Note that this can lead to state events
when the mass flow rate approaches zero,
which can increase computing time.
</p>
<h4>Notes</h4>
<p>
For more detailed models that compute the actual flow friction,
models from the package
<a href=\"modelica://Modelica.Fluid\">
Modelica.Fluid</a>
can be used and combined with models from the
<code>Buildings</code> library.
</p>
<p>
For a model that uses <code>dp_nominal</code> as a parameter rather than
geoemetric data, use
<a href=\"modelica://Buildings.Fluid.FixedResistances.PressureDrop\">
Buildings.Fluid.FixedResistances.PressureDrop</a>.
</p>
<h4>Implementation</h4>
<p>
The pressure drop is computed by calling a function in the package
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels\">
Buildings.Fluid.BaseClasses.FlowModels</a>,
This package contains regularized implementations of the equation
</p>
<p align=\"center\" style=\"font-style:italic;\">
  m = sign(&Delta;p) k  &radic;<span style=\"text-decoration:overline;\">&nbsp;&Delta;p &nbsp;</span>
</p>
<p>
and its inverse function.
</p>
<p>
To decouple the energy equation from the mass equations,
the pressure drop is a function of the mass flow rate,
and not the volume flow rate.
This leads to simpler equations.
</p>
</html>",           revisions="<html>
<ul>
<li>
December 1, 2016, by Michael Wetter:<br/>
First implementation for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/480\">#480</a>.
</li>
</ul>
</html>"),  Icon(graphics={Text(
                    extent={{-40,18},{38,-20}},
                    lineColor={255,255,255},
                    textString="dh")}));
          end HydraulicDiameter;

          model PlugFlowPipe
            "Pipe model using spatialDistribution for temperature delay"
            extends Buildings.Fluid.Interfaces.PartialTwoPortVector;

            constant Boolean homotopyInitialization = true "= true, use homotopy method"
              annotation(HideResult=true);

            parameter Boolean from_dp=false
              "= true, use m_flow = f(dp) else dp = f(m_flow)"
              annotation (Dialog(tab="Advanced"));

            parameter Modelica.SIunits.Length dh=sqrt(4*m_flow_nominal/rho_default/v_nominal/Modelica.Constants.pi)
              "Hydraulic diameter (assuming a round cross section area)"
              annotation (Dialog(group="Material"));

            parameter Modelica.SIunits.Velocity v_nominal = 1.5
              "Velocity at m_flow_nominal (used to compute default value for hydraulic diameter dh)"
              annotation(Dialog(group="Nominal condition"));

            parameter Real ReC=4000
              "Reynolds number where transition to turbulent starts";

            parameter Modelica.SIunits.Height roughness=2.5e-5
              "Average height of surface asperities (default: smooth steel pipe)"
              annotation (Dialog(group="Material"));

            parameter Modelica.SIunits.Length length "Pipe length"
              annotation (Dialog(group="Material"));

            parameter Modelica.SIunits.MassFlowRate m_flow_nominal
              "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

            parameter Modelica.SIunits.MassFlowRate m_flow_small = 1E-4*abs(
              m_flow_nominal) "Small mass flow rate for regularization of zero flow"
              annotation (Dialog(tab="Advanced"));

            parameter Modelica.SIunits.Length dIns
              "Thickness of pipe insulation, used to compute R"
              annotation (Dialog(group="Thermal resistance"));

            parameter Modelica.SIunits.ThermalConductivity kIns
              "Heat conductivity of pipe insulation, used to compute R"
              annotation (Dialog(group="Thermal resistance"));

            parameter Modelica.SIunits.SpecificHeatCapacity cPip=2300
              "Specific heat of pipe wall material. 2300 for PE, 500 for steel"
              annotation (Dialog(group="Material"));

            parameter Modelica.SIunits.Density rhoPip(displayUnit="kg/m3")=930
              "Density of pipe wall material. 930 for PE, 8000 for steel"
              annotation (Dialog(group="Material"));

            parameter Modelica.SIunits.Length thickness = 0.0035
              "Pipe wall thickness"
              annotation (Dialog(group="Material"));

            parameter Modelica.SIunits.Temperature T_start_in(start=Medium.T_default)=
              Medium.T_default "Initialization temperature at pipe inlet"
              annotation (Dialog(tab="Initialization"));
            parameter Modelica.SIunits.Temperature T_start_out(start=Medium.T_default)=
              T_start_in "Initialization temperature at pipe outlet"
              annotation (Dialog(tab="Initialization"));
            parameter Boolean initDelay(start=false)=true
              "Initialize delay for a constant mass flow rate if true, otherwise start from 0"
              annotation (Dialog(tab="Initialization"));
            parameter Modelica.SIunits.MassFlowRate m_flow_start=m_flow_nominal
                                                                   "Initial value of mass flow rate through pipe"
              annotation (Dialog(tab="Initialization", enable=initDelay));

            parameter Real R(unit="(m.K)/W")=1/(kIns*2*Modelica.Constants.pi/
              Modelica.Math.log((dh/2 + thickness + dIns)/(dh/2 + thickness)))
              "Thermal resistance per unit length from fluid to boundary temperature"
              annotation (Dialog(group="Thermal resistance"));

            parameter Real fac=1
              "Factor to take into account flow resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal";

            parameter Boolean linearized = false
              "= true, use linear relation between m_flow and dp for any flow rate"
              annotation(Evaluate=true, Dialog(tab="Advanced"));

            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
              "Heat transfer to or from surroundings (heat loss from pipe results in a positive heat flow)"
              annotation (Placement(transformation(extent={{-10,90},{10,110}})));

            Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore cor(
              redeclare final package Medium = Medium,
              final dh=dh,
              final v_nominal=v_nominal,
              final length=length,
              final C=C,
              final R=R,
              final m_flow_small=m_flow_small,
              final m_flow_nominal=m_flow_nominal,
              final T_start_in=T_start_in,
              final T_start_out=T_start_out,
              final m_flow_start=m_flow_start,
              final initDelay=initDelay,
              final from_dp=from_dp,
              final fac=fac,
              final ReC=ReC,
              final thickness=thickness,
              final roughness=roughness,
              final allowFlowReversal=allowFlowReversal,
              final homotopyInitialization=homotopyInitialization,
              final linearized=linearized) "Describing the pipe behavior" annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

            // In the volume, below, we scale down V and use
            // mSenFac. Otherwise, for air, we would get very large volumes
            // which affect the delay of water vapor and contaminants.
            // See also Buildings.Fluid.FixedResistances.Validation.PlugFlowPipes.TransportWaterAir
            // for why mSenFac is 10 and not 1000, as this gives more reasonable
            // temperature step response
            Fluid.MixingVolumes.MixingVolume vol(
              redeclare final package Medium = Medium,
              final m_flow_nominal=m_flow_nominal,
              final V=if rho_default > 500 then VEqu else VEqu/1000,
              final nPorts=nPorts + 1,
              final T_start=T_start_out,
              final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
              final mSenFac = if rho_default > 500 then 1 else 10)
              "Control volume connected to ports_b. Represents equivalent pipe wall thermal capacity."
              annotation (Placement(transformation(extent={{60,20},{80,40}})));

          protected
            parameter Modelica.SIunits.HeatCapacity CPip=
              length*((dh + 2*thickness)^2 - dh^2)*Modelica.Constants.pi/4*cPip*rhoPip "Heat capacity of pipe wall";

            final parameter Modelica.SIunits.Volume VEqu=CPip/(rho_default*cp_default)
              "Equivalent water volume to represent pipe wall thermal inertia";

            parameter Medium.ThermodynamicState sta_default=Medium.setState_pTX(
                T=Medium.T_default,
                p=Medium.p_default,
                X=Medium.X_default) "Default medium state";

            parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
                Medium.specificHeatCapacityCp(state=sta_default)
              "Heat capacity of medium";

            parameter Real C(unit="J/(K.m)")=
              rho_default*Modelica.Constants.pi*(dh/2)^2*cp_default
              "Thermal capacity per unit length of water in pipe";

            parameter Modelica.SIunits.Density rho_default=Medium.density_pTX(
                p=Medium.p_default,
                T=Medium.T_default,
                X=Medium.X_default)
              "Default density (e.g., rho_liquidWater = 995, rho_air = 1.2)"
              annotation (Dialog(group="Advanced"));

          initial equation
            assert(homotopyInitialization, "In " + getInstanceName() +
              ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
              level = AssertionLevel.warning);

          equation
            for i in 1:nPorts loop
              connect(vol.ports[i + 1], ports_b[i])
              annotation (Line(points={{70,20},{72,20},{72,6},{72,0},{100,0}},
                  color={0,127,255}));
            end for;
            connect(cor.heatPort, heatPort)
              annotation (Line(points={{0,10},{0,10},{0,100}}, color={191,0,0}));

            connect(cor.port_b, vol.ports[1])
              annotation (Line(points={{10,0},{70,0},{70,20}}, color={0,127,255}));

            connect(cor.port_a, port_a)
              annotation (Line(points={{-10,0},{-56,0},{-100,0}}, color={0,127,255}));
            annotation (
              Line(points={{70,20},{72,20},{72,0},{100,0}}, color={0,127,255}),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                      100,100}})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                      100}}), graphics={
                  Rectangle(
                    extent={{-100,40},{100,-40}},
                    lineColor={0,0,0},
                    fillPattern=FillPattern.HorizontalCylinder,
                    fillColor={192,192,192}),
                  Rectangle(
                    extent={{-100,30},{100,-30}},
                    lineColor={0,0,0},
                    fillPattern=FillPattern.HorizontalCylinder,
                    fillColor={0,127,255}),
                  Rectangle(
                    extent={{-100,50},{100,40}},
                    lineColor={175,175,175},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Backward),
                  Rectangle(
                    extent={{-100,-40},{100,-50}},
                    lineColor={175,175,175},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Backward),
                  Polygon(
                    points={{0,90},{40,62},{20,62},{20,38},{-20,38},{-20,62},{-40,62},{0,
                        90}},
                    lineColor={0,0,0},
                    fillColor={238,46,47},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{-30,30},{28,-30}},
                    lineColor={0,0,0},
                    fillPattern=FillPattern.HorizontalCylinder,
                    fillColor={215,202,187}),
                  Text(
                    extent={{-100,-72},{100,-88}},
                    lineColor={0,0,0},
                    textString="L = %length
d = %dh")}),  Documentation(revisions="<html>
<ul>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">Buildings, #1341</a>.
</li>
<li>
March 6, 2020, by Jelger Jansen:<br/>
Revised calculation of thermal resistance <code>R</code>
by using correct radiuses.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1310\">#1310</a>.
</li>
<li>
October 23, 2017, by Michael Wetter:<br/>
Revised variable names and documentation to follow guidelines.
Corrected malformed hyperlinks.
</li>
<li>
July 4, 2016 by Bram van der Heijde:<br/>
Introduce <code>pipVol</code>.
</li>
<li>
October 10, 2015 by Marcus Fuchs:<br/>
Copy Icon from KUL implementation and rename model.
Replace resistance and temperature delay by an adiabatic pipe.
</li>
<li>September, 2015 by Marcus Fuchs:<br/>
First implementation.
</li>
</ul>
</html>",           info="<html>
<p>
Pipe with heat loss using the time delay based heat losses and transport
of the fluid using a plug flow model, applicable for simulation of long
pipes such as in district heating and cooling systems.</p>
<p>
This model takes into account transport delay along the pipe length idealized
as a plug flow.
The model also includes thermal inertia of the pipe wall.
</p>
<h4>Implementation</h4>
<p>Heat losses are implemented by
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss</a>
at each end of the pipe (see
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore</a>).
Depending on the flow direction, the temperature difference due to heat losses
is subtracted at the right fluid port.
</p>
<p>
The pressure drop is implemented using
<a href=\"modelica://Buildings.Fluid.FixedResistances.HydraulicDiameter\">
Buildings.Fluid.FixedResistances.HydraulicDiameter</a>.
</p>
<p>
The thermal capacity of the pipe wall is implemented as a mixing volume
of the fluid in the pipe, of which the thermal capacity
is equal to that of the pipe wall material.
In addition, this mixing volume allows the hydraulic separation of subsequent pipes.
Thanks to the vectorized implementation of the (design) outlet port,
splits and junctions of pipes can be handled in a numerically efficient way.
<br/>
This mixing volume is not present in the
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore\">PlugFlowCore</a> model,
which can be used in cases where mixing volumes at pipe junctions need to
be added manually.
</p>
<h4>Assumptions</h4>
<ul>
<li>
Heat losses are for steady-state operation.
</li>
<li>
The axial heat diffusion in the fluid, the pipe wall and the ground are neglected.
</li>
<li>
The boundary temperature is uniform.
</li>
<li>
The thermal inertia of the pipe wall material is lumped on the side of the pipe
that is connected to <code>ports_b</code>.
</li>
</ul>
<h4>References</h4>
<p>
Full details on the model implementation and experimental validation can be found
in:
</p>
<p>
van der Heijde, B., Fuchs, M., Ribas Tugores, C., Schweiger, G., Sartor, K.,
Basciotti, D., M&uuml;ller, D., Nytsch-Geusen, C., Wetter, M. and Helsen, L.
(2017).<br/>
Dynamic equation-based thermo-hydraulic pipe model for district heating and
cooling systems.<br/>
<i>Energy Conversion and Management</i>, vol. 151, p. 158-169.
<a href=\"https://doi.org/10.1016/j.enconman.2017.08.072\">doi:
10.1016/j.enconman.2017.08.072</a>.</p>
</html>"));
          end PlugFlowPipe;

          model PressureDrop
            "Fixed flow resistance with dp and m_flow as parameter"
            extends Buildings.Fluid.BaseClasses.PartialResistance(final m_flow_turbulent=if computeFlowResistance then deltaM*m_flow_nominal_pos else 0);

            parameter Real deltaM(min=1E-6) = 0.3
              "Fraction of nominal mass flow rate where transition to turbulent occurs"
                 annotation(Evaluate=true,
                            Dialog(group = "Transition to laminar",
                                   enable = not linearized));

            final parameter Real k = if computeFlowResistance then
                  m_flow_nominal_pos / sqrt(dp_nominal_pos) else 0
              "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
          protected
            final parameter Boolean computeFlowResistance=(dp_nominal_pos > Modelica.Constants.eps)
              "Flag to enable/disable computation of flow resistance"
             annotation(Evaluate=true);
            final parameter Real coeff=
              if linearized and computeFlowResistance
              then if from_dp then k^2/m_flow_nominal_pos else m_flow_nominal_pos/k^2
              else 0
              "Precomputed coefficient to avoid division by parameter";
          initial equation
           if computeFlowResistance then
             assert(m_flow_turbulent > 0, "m_flow_turbulent must be bigger than zero.");
           end if;

           assert(m_flow_nominal_pos > 0, "m_flow_nominal_pos must be non-zero. Check parameters.");
          equation
            // Pressure drop calculation
            if computeFlowResistance then
              if linearized then
                if from_dp then
                  m_flow = dp*coeff;
                else
                  dp = m_flow*coeff;
                end if;
              else
                if homotopyInitialization then
                  if from_dp then
                    m_flow = homotopy(actual=Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp(
                      dp=dp,
                      k=k,
                      m_flow_turbulent=m_flow_turbulent), simplified=m_flow_nominal_pos*dp/dp_nominal_pos);
                  else
                    dp = homotopy(actual=Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow(
                      m_flow=m_flow,
                      k=k,
                      m_flow_turbulent=m_flow_turbulent), simplified=dp_nominal_pos*m_flow/m_flow_nominal_pos);
                   end if;  // from_dp
                else // do not use homotopy
                  if from_dp then
                    m_flow = Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp(
                      dp=dp,
                      k=k,
                      m_flow_turbulent=m_flow_turbulent);
                  else
                    dp = Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow(
                      m_flow=m_flow,
                      k=k,
                      m_flow_turbulent=m_flow_turbulent);
                  end if;  // from_dp
                end if; // homotopyInitialization
              end if; // linearized
            else // do not compute flow resistance
              dp = 0;
            end if;  // computeFlowResistance

            annotation (defaultComponentName="res",
          Documentation(info="<html>
<p>
Model of a flow resistance with a fixed flow coefficient.
The mass flow rate is
</p>
<p align=\"center\" style=\"font-style:italic;\">
m&#775; = k
&radic;<span style=\"text-decoration:overline;\">&Delta;P</span>,
</p>
<p>
where
<i>k</i> is a constant and
<i>&Delta;P</i> is the pressure drop.
The constant <i>k</i> is equal to
<code>k=m_flow_nominal/sqrt(dp_nominal)</code>,
where <code>m_flow_nominal</code> and <code>dp_nominal</code>
are parameters.
</p>
<h4>Assumptions</h4>
<p>
In the region
<code>abs(m_flow) &lt; m_flow_turbulent</code>,
the square root is replaced by a differentiable function
with finite slope.
The value of <code>m_flow_turbulent</code> is
computed as
<code>m_flow_turbulent = deltaM * abs(m_flow_nominal)</code>,
where <code>deltaM=0.3</code> and
<code>m_flow_nominal</code> are parameters that can be set by the user.
</p>
<p>
The figure below shows the pressure drop for the parameters
<code>m_flow_nominal=5</code> kg/s,
<code>dp_nominal=10</code> Pa and
<code>deltaM=0.3</code>.
</p>
<p align=\"center\">
<img alt=\"image\" src=\"modelica://Buildings/Resources/Images/Fluid/FixedResistances/PressureDrop.png\"/>
</p>
<h4>Important parameters</h4>
<p>
The parameter <code>from_dp</code> is used to determine
whether the mass flow rate is computed as a function of the
pressure drop (if <code>from_dp=true</code>), or vice versa.
This setting can affect the size of the nonlinear system of equations.
</p>
<p>
If the parameter <code>linearized</code> is set to <code>true</code>,
then the pressure drop is computed as a linear function of the
mass flow rate.
</p>
<p>
Setting <code>allowFlowReversal=false</code> can lead to simpler
equations. However, this should only be set to <code>false</code>
if one can guarantee that the flow never reverses its direction.
This can be difficult to guarantee, as pressure imbalance after
the initialization, or due to medium expansion and contraction,
can lead to reverse flow.
</p>
<p>
If the parameter
<code>show_T</code> is set to <code>true</code>,
then the model will compute the
temperature at its ports. Note that this can lead to state events
when the mass flow rate approaches zero,
which can increase computing time.
</p>
<h4>Notes</h4>
<p>
For more detailed models that compute the actual flow friction,
models from the package
<a href=\"modelica://Modelica.Fluid\">
Modelica.Fluid</a>
can be used and combined with models from the
<code>Buildings</code> library.
</p>
<p>
For a model that uses the hydraulic parameter and flow velocity at nominal conditions
as a parameter, use
<a href=\"modelica://Buildings.Fluid.FixedResistances.HydraulicDiameter\">
Buildings.Fluid.FixedResistances.HydraulicDiameter</a>.
</p>
<h4>Implementation</h4>
<p>
The pressure drop is computed by calling a function in the package
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels\">
Buildings.Fluid.BaseClasses.FlowModels</a>,
This package contains regularized implementations of the equation
</p>
<p align=\"center\" style=\"font-style:italic;\">
  m = sign(&Delta;p) k  &radic;<span style=\"text-decoration:overline;\">&nbsp;&Delta;p &nbsp;</span>
</p>
<p>
and its inverse function.
</p>
<p>
To decouple the energy equation from the mass equations,
the pressure drop is a function of the mass flow rate,
and not the volume flow rate.
This leads to simpler equations.
</p>
</html>",           revisions="<html>
<ul>
<li>
September 21, 2018, by Michael Wetter:<br/>
Decrease value of <code>deltaM(min=...)</code> attribute.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1026\">#1026</a>.
</li>
<li>
February 3, 2018, by Filip Jorissen:<br/>
Revised implementation of pressure drop equation
such that it depends on <code>from_dp</code>
when <code>linearized=true</code>.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/884\">#884</a>.
</li>
<li>
December 1, 2016, by Michael Wetter:<br/>
Simplified model by removing the geometry dependent parameters into the new
model
<a href=\"modelica://Buildings.Fluid.FixedResistances.HydraulicDiameter\">
Buildings.Fluid.FixedResistances.HydraulicDiameter</a>.
</li>
<li>
November 23, 2016, by Filip Jorissen:<br/>
Removed <code>dp_nominal</code> and
<code>m_flow_nominal</code> labels from icon.
</li>
<li>
October 14, 2016, by Michael Wetter:<br/>
Updated comment for parameter <code>use_dh</code>.
</li>
<li>
November 26, 2014, by Michael Wetter:<br/>
Added the required <code>annotation(Evaluate=true)</code> so
that the system of nonlinear equations in
<a href=\"modelica://Buildings.Fluid.FixedResistances.Validation.PressureDropsExplicit\">
Buildings.Fluid.FixedResistances.Validation.PressureDropsExplicit</a>
remains the same.
</li>
<li>
November 20, 2014, by Michael Wetter:<br/>
Rewrote the warning message using an <code>assert</code> with
<code>AssertionLevel.warning</code>
as this is the proper way to write warnings in Modelica.
</li>
<li>
August 5, 2014, by Michael Wetter:<br/>
Corrected error in documentation of computation of <code>k</code>.
</li>
<li>
May 29, 2014, by Michael Wetter:<br/>
Removed undesirable annotation <code>Evaluate=true</code>.
</li>
<li>
October 8, 2013, by Michael Wetter:<br/>
Removed parameter <code>show_V_flow</code>.
</li>
<li>
December 14, 2012 by Michael Wetter:<br/>
Renamed protected parameters for consistency with the naming conventions.
</li>
<li>
January 16, 2012 by Michael Wetter:<br/>
To simplify object inheritance tree, revised base classes
<code>Buildings.Fluid.BaseClasses.PartialResistance</code>,
<code>Buildings.Fluid.Actuators.BaseClasses.PartialTwoWayValve</code>,
<code>Buildings.Fluid.Actuators.BaseClasses.PartialDamperExponential</code>,
<code>Buildings.Fluid.Actuators.BaseClasses.PartialActuator</code>
and model
<code>Buildings.Fluid.FixedResistances.PressureDrop</code>.
</li>
<li>
May 30, 2008 by Michael Wetter:<br/>
Added parameters <code>use_dh</code> and <code>deltaM</code> for easier parameterization.
</li>
<li>
July 20, 2007 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
          end PressureDrop;

          package BaseClasses "Package with base classes for Buildings.Fluid.FixedResistances"
            extends Modelica.Icons.BasesPackage;

            model PlugFlow
              "Lossless pipe model with spatialDistribution plug flow implementation"
              extends Buildings.Fluid.Interfaces.PartialTwoPort;

              parameter Modelica.SIunits.Length dh
                "Hydraulic diameter (assuming a round cross section area)";
              parameter Modelica.SIunits.Length length(min=0) "Pipe length";
              final parameter Modelica.SIunits.Area A=Modelica.Constants.pi*(dh/2)^2
                "Cross-sectional area of pipe";

              parameter Medium.MassFlowRate m_flow_small
                "Small mass flow rate for regularization of zero flow"
                annotation(Dialog(tab = "Advanced"));
              parameter Modelica.SIunits.Temperature T_start_in=Medium.T_default
                "Initial temperature in pipe at inlet"
                annotation (Dialog(group="Initialization"));
              parameter Modelica.SIunits.Temperature T_start_out=Medium.T_default
                "Initial temperature in pipe at outlet"
                annotation (Dialog(group="Initialization"));

              Modelica.SIunits.Length x
                "Spatial coordinate for spatialDistribution operator";
              Modelica.SIunits.Velocity v "Flow velocity of medium in pipe";

              Modelica.SIunits.VolumeFlowRate V_flow=
                  port_a.m_flow/Modelica.Fluid.Utilities.regStep(port_a.m_flow,
                              Medium.density(
                                Medium.setState_phX(
                                  p = port_a.p,
                                  h = inStream(port_a.h_outflow),
                                  X = inStream(port_a.Xi_outflow))),
                              Medium.density(
                                   Medium.setState_phX(
                                     p = port_b.p,
                                     h = inStream(port_b.h_outflow),
                                     X = inStream(port_b.Xi_outflow))),
                              m_flow_small)
                "Volume flow rate at inflowing port (positive when flow from port_a to port_b)";

            protected
              parameter Modelica.SIunits.SpecificEnthalpy h_ini_in=Medium.specificEnthalpy(
                  Medium.setState_pTX(
                  T=T_start_in,
                  p=Medium.p_default,
                  X=Medium.X_default)) "For initialization of spatialDistribution inlet";

              parameter Modelica.SIunits.SpecificEnthalpy h_ini_out=Medium.specificEnthalpy(
                   Medium.setState_pTX(
                  T=T_start_out,
                  p=Medium.p_default,
                  X=Medium.X_default)) "For initialization of spatialDistribution outlet";

              Medium.MassFraction Xi_inflow_a[Medium.nXi] = inStream(port_a.Xi_outflow)
                "Independent mixture mass fractions m_i/m close to the connection point flow into the component";
              Medium.MassFraction Xi_inflow_b[Medium.nXi] = inStream(port_b.Xi_outflow)
                "Independent mixture mass fractions m_i/m close to the connection point flow into the component";
              Medium.ExtraProperty C_inflow_a[Medium.nC] = inStream(port_a.C_outflow)
                "Properties c_i/m close to the connection point if flow into the component";
              Medium.ExtraProperty C_inflow_b[Medium.nC] = inStream(port_b.C_outflow)
                "Properties c_i/m close to the connection point if flow into the component";
            initial equation
              x = 0;
            equation
              // No pressure drop
              port_a.p = port_b.p;

              // Mass balance (no storage)
              port_a.m_flow + port_b.m_flow = 0;

              der(x) = v;
              v = V_flow/A;

              (port_a.h_outflow, port_b.h_outflow) = spatialDistribution(
                inStream(port_a.h_outflow),
                inStream(port_b.h_outflow),
                x/length,
                v >= 0,
                {0.0, 1.0},
                {h_ini_in, h_ini_out});

              // Transport of substances
              for i in 1:Medium.nXi loop
              (port_a.Xi_outflow[i], port_b.Xi_outflow[i]) = spatialDistribution(
                Xi_inflow_a[i],
                Xi_inflow_b[i],
                x/length,
                v >= 0,
                {0.0, 1.0},
                {0.01, 0.01});
              end for;

              for i in 1:Medium.nC loop
              (port_a.C_outflow[i], port_b.C_outflow[i]) = spatialDistribution(
                C_inflow_a[i],
                C_inflow_b[i],
                x/length,
                v >= 0,
                {0.0, 1.0},
                {0, 0});
              end for;

              annotation (
                Icon(graphics={
                    Line(
                      points={{-72,-28}},
                      color={0,0,0},
                      pattern=LinePattern.Dash,
                      thickness=0.5,
                      smooth=Smooth.None),
                    Rectangle(
                      extent={{-100,60},{100,-60}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.HorizontalCylinder,
                      fillColor={192,192,192}),
                    Rectangle(
                      extent={{-100,50},{100,-48}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.HorizontalCylinder,
                      fillColor={217,236,256}),
                    Rectangle(
                      extent={{-20,50},{20,-48}},
                      lineColor={175,175,175},
                      fillPattern=FillPattern.HorizontalCylinder,
                      fillColor={175,175,175})}),
                Documentation(revisions="<html>
<ul>
<li>
October 20, 2017, by Michael Wetter:<br/>
Deleted various parameters and variables that were not used.
<br/>
Revised documentation to follow the guidelines.
</li>
<li>
May 19, 2016 by Marcus Fuchs:<br/>
Remove condition on <code>show_V_flow</code> for calculation of
<code>V_flow</code> to conform with pedantic checking.
</li>
<li>
October 10, 2015 by Marcus Fuchs:<br/>
Copy Icon from KUL implementation and rename model.
</li>
<li>
June 23, 2015 by Marcus Fuchs:<br/>
First implementation.
</li>
</ul>
</html>",             info="<html>
<p>
Model that computes the temperature propagation of
a fluid flow through a pipe, idealized as a plug flow.
</p>
<h4>Main equation</h4>
<p>
The transport delay is computed using the one-dimensional wave equation
without source or sink terms,
<p align=\"center\" style=\"font-style:italic;\">
&part;z(x,t)/&part;t + v(t) &part;z(x,t)/&part;x = 0,
</p>
<p>where <i>z(x,t)</i> is the spatial distribution as a function of time of any
property <i>z</i> of the fluid.
For the temperature propagation, <i>z </i>will be replaced by <i>T</i>.
</p>
<h4>Assumptions</h4>
<p>
This model is based on the following assumptions:
</p>
<ul>
<li>
Axial diffusion in water is assumed to be negligibe.
</li>
<li>
The water temperature is assumed uniform in a cross section.
</li>
</ul>
</html>"));
            end PlugFlow;

            model PlugFlowCore
              "Pipe model using spatialDistribution for temperature delay with modified delay tracker"
              extends Buildings.Fluid.Interfaces.PartialTwoPort;

              constant Boolean homotopyInitialization = true "= true, use homotopy method"
                annotation(HideResult=true);

              parameter Modelica.SIunits.Length dh
                "Hydraulic diameter (assuming a round cross section area)";

              parameter Modelica.SIunits.Velocity v_nominal
                "Velocity at m_flow_nominal (used to compute default value for hydraulic diameter dh)"
                annotation(Dialog(group="Nominal condition"));

              parameter Modelica.SIunits.Length length(min=0) "Pipe length";

              parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
                "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

              parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(
                m_flow_nominal) "Small mass flow rate for regularization of zero flow"
                annotation (Dialog(tab="Advanced"));

              parameter Modelica.SIunits.Height roughness=2.5e-5
                "Average height of surface asperities (default: smooth steel pipe)"
                annotation (Dialog(group="Geometry"));

              parameter Real R(unit="(m.K)/W")
                "Thermal resistance per unit length from fluid to boundary temperature";

              parameter Real C(unit="J/(K.m)")
                "Thermal capacity per unit length of pipe";

              parameter Real fac=1
                "Factor to take into account flow resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal";

              parameter Boolean from_dp=false
                "= true, use m_flow = f(dp) else dp = f(m_flow)"
                annotation (Evaluate=true, Dialog(tab="Advanced"));
              parameter Modelica.SIunits.Length thickness(min=0) "Pipe wall thickness";

              parameter Modelica.SIunits.Temperature T_start_in=Medium.T_default
                "Initialization temperature at pipe inlet"
                annotation (Dialog(tab="Initialization"));
              parameter Modelica.SIunits.Temperature T_start_out=Medium.T_default
                "Initialization temperature at pipe outlet"
                annotation (Dialog(tab="Initialization"));
              parameter Boolean initDelay=true
                "Initialize delay for a constant mass flow rate if true, otherwise start from 0"
                annotation (Dialog(tab="Initialization"));
              parameter Modelica.SIunits.MassFlowRate m_flow_start=m_flow_nominal(min=0)
                annotation (Dialog(tab="Initialization", enable=initDelay));

              parameter Real ReC=4000
                "Reynolds number where transition to turbulent starts";

              parameter Boolean linearized = false
                "= true, use linear relation between m_flow and dp for any flow rate"
                annotation(Evaluate=true, Dialog(tab="Advanced"));

              Buildings.Fluid.FixedResistances.HydraulicDiameter res(
                redeclare final package Medium = Medium,
                final dh=dh,
                final m_flow_nominal=m_flow_nominal,
                final from_dp=from_dp,
                final length=length,
                final roughness=roughness,
                final fac=fac,
                final ReC=ReC,
                final v_nominal=v_nominal,
                final allowFlowReversal=allowFlowReversal,
                final show_T=false,
                final homotopyInitialization=homotopyInitialization,
                final linearized=linearized,
                dp(nominal=fac*200*length)) "Pressure drop calculation for this pipe" annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

              Buildings.Fluid.FixedResistances.BaseClasses.PlugFlow del(
                redeclare final package Medium = Medium,
                final m_flow_small=m_flow_small,
                final dh=dh,
                final length=length,
                final allowFlowReversal=allowFlowReversal,
                final T_start_in=T_start_in,
                final T_start_out=T_start_out) "Model for temperature wave propagation" annotation (Placement(transformation(extent={{20,-10},{40,10}})));

              Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss heaLos_a(
                redeclare final package Medium = Medium,
                final C=C,
                final R=R,
                final m_flow_small=m_flow_small,
                final T_start=T_start_in,
                final m_flow_nominal=m_flow_nominal,
                final m_flow_start=m_flow_start,
                final show_T=false,
                final show_V_flow=false) "Heat loss for flow from port_b to port_a" annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));

              Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss heaLos_b(
                redeclare final package Medium = Medium,
                final C=C,
                final R=R,
                final m_flow_small=m_flow_small,
                final T_start=T_start_out,
                final m_flow_nominal=m_flow_nominal,
                final m_flow_start=m_flow_start,
                final show_T=false,
                final show_V_flow=false) "Heat loss for flow from port_a to port_b" annotation (Placement(transformation(extent={{60,-10},{80,10}})));
              Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare final package Medium = Medium) "Mass flow sensor" annotation (Placement(transformation(extent={{-50,10},{-30,-10}})));
              Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowTransportDelay timDel(
                final length=length,
                final dh=dh,
                final rho=rho_default,
                final initDelay=initDelay,
                final m_flow_nominal=m_flow_nominal,
                final m_flow_start=m_flow_start) "Time delay" annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
              Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
                "Heat port to connect environment (positive heat flow for heat loss to surroundings)"
                annotation (Placement(transformation(extent={{-10,90},{10,110}})));

            protected
              parameter Modelica.SIunits.Density rho_default=Medium.density_pTX(
                  p=Medium.p_default,
                  T=Medium.T_default,
                  X=Medium.X_default)
                "Default density (e.g., rho_liquidWater = 995, rho_air = 1.2)"
                annotation (Dialog(group="Advanced"));

            initial equation
              assert(homotopyInitialization, "In " + getInstanceName() +
                ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
                level = AssertionLevel.warning);

            equation
              connect(senMasFlo.m_flow, timDel.m_flow) annotation (Line(
                  points={{-40,-11},{-40,-40},{-12,-40}},
                  color={0,0,127},
                  smooth=Smooth.None));
              connect(heaLos_a.heatPort, heatPort) annotation (Line(points={{-70,10},{-70,40},
                      {0,40},{0,100}}, color={191,0,0}));
              connect(heaLos_b.heatPort, heatPort) annotation (Line(points={{70,10},{70,40},
                      {0,40},{0,100}}, color={191,0,0}));

              connect(timDel.tauRev, heaLos_a.tau) annotation (Line(points={{11,-36},{50,-36},
                      {50,28},{-64,28},{-64,10}}, color={0,0,127}));
              connect(timDel.tau, heaLos_b.tau) annotation (Line(points={{11,-44},{54,-44},
                      {54,28},{64,28},{64,10}}, color={0,0,127}));

              connect(port_a, heaLos_a.port_b)
                annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
              connect(heaLos_a.port_a, senMasFlo.port_a)
                annotation (Line(points={{-60,0},{-50,0}}, color={0,127,255}));
              connect(heaLos_b.port_b, port_b)
                annotation (Line(points={{80,0},{100,0}}, color={0,127,255}));
              connect(del.port_a, res.port_b)
                annotation (Line(points={{20,0},{0,0}}, color={0,127,255}));
              connect(senMasFlo.port_b, res.port_a)
                annotation (Line(points={{-30,0},{-20,0}}, color={0,127,255}));
              connect(heaLos_b.port_a, del.port_b)
                annotation (Line(points={{60,0},{40,0}}, color={0,127,255}));
              annotation (
                Line(points={{70,20},{72,20},{72,0},{100,0}}, color={0,127,255}),
                Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                        100}})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                    graphics={
                    Rectangle(
                      extent={{-100,40},{100,-40}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.HorizontalCylinder,
                      fillColor={192,192,192}),
                    Rectangle(
                      extent={{-100,30},{100,-30}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.HorizontalCylinder,
                      fillColor={0,127,255}),
                    Rectangle(
                      extent={{-100,50},{100,40}},
                      lineColor={175,175,175},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Backward),
                    Rectangle(
                      extent={{-100,-40},{100,-50}},
                      lineColor={175,175,175},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Backward),
                    Polygon(
                      points={{0,100},{40,62},{20,62},{20,38},{-20,38},{-20,62},{-40,62},{0,
                          100}},
                      lineColor={0,0,0},
                      fillColor={238,46,47},
                      fillPattern=FillPattern.Solid),
                    Rectangle(
                      extent={{-30,30},{28,-30}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.HorizontalCylinder,
                      fillColor={215,202,187})}),
                Documentation(revisions="<html>
<ul>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">Buildings, #1341</a>.
</li>
<li>
October 20, 2017, by Michael Wetter:<br/>
Replaced model that lumps flow resistance and transport delays
with two separate models, as these are physically distinct processes.
This also avoids one more layer of models.
<br/>
Revised variable names and documentation to follow guidelines.
</li>
<li>
July 4, 2016 by Bram van der Heijde:<br/>
Introduce <code>pipVol</code>.
</li>
<li>
October 10, 2015 by Marcus Fuchs:<br/>
Copy Icon from KUL implementation and rename model.
Replace resistance and temperature delay by an adiabatic pipe.
</li>
<li>
September, 2015 by Marcus Fuchs:<br/>First implementation.
</li>
</ul>
</html>",             info="<html>
<p>
Pipe with heat loss using the time delay based heat losses and plug flow
for the transport delay of the fluid.
</p>
<h4>Implementation</h4>
<p>
The
<code>spatialDistribution</code> operator is used for the temperature wave propagation
through the length of the pipe. This operator is contained in
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlow\">BaseClasses.PlugFlow</a>.
</p>
<p>
This model does not include thermal inertia of the pipe wall.
The wall inertia is implemented in
<a href=\"modelica://Buildings.Fluid.FixedResistances.PlugFlowPipe\">PlugFlowPipe</a>, which uses this model.
<br/>
The removal of the thermal inertia with a mixing volume can be desirable in the
case where mixing volumes are added manually at the pipe junctions.
</p>
<p>
The model
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss\">
PlugFlowHeatLoss</a>
implements a heat loss in design direction, but leaves the enthalpy unchanged
in opposite flow direction. Therefore it is used in front of and behind the time delay.
</p>
<h4>References</h4>
<p>
Full details on the model implementation and experimental validation can be found
in:
</p>
<p>
van der Heijde, B., Fuchs, M., Ribas Tugores, C., Schweiger, G., Sartor, K., Basciotti, D., M&uuml;ller,
D., Nytsch-Geusen, C., Wetter, M. and Helsen, L. (2017).<br/>
Dynamic equation-based thermo-hydraulic pipe model for district heating and cooling systems.<br/>
<i>Energy Conversion and Management</i>, vol. 151, p. 158-169.
<a href=\"https://doi.org/10.1016/j.enconman.2017.08.072\">doi: 10.1016/j.enconman.2017.08.072</a>.</p>
</html>"));
            end PlugFlowCore;

            model PlugFlowHeatLoss
              "Heat loss model for pipe with delay as an input variable"
              extends Fluid.Interfaces.PartialTwoPortTransport(
                final allowFlowReversal=true,
                final dp_start=0);
                // allowFlowReversal set to true because this model is used for inlet and outlets

              parameter Real C(unit="J/(K.m)")
                "Thermal capacity per unit length of pipe";
              parameter Real R(unit="(m.K)/W")
                "Thermal resistance per unit length from fluid to boundary temperature";

              parameter Modelica.SIunits.MassFlowRate m_flow_nominal "Nominal mass flow rate";
              parameter Modelica.SIunits.Temperature T_start
                "Initial output temperature";

              final parameter Modelica.SIunits.Time tau_char=R*C "Characteristic delay time";

              Modelica.Blocks.Interfaces.RealInput tau(unit="s") "Time delay at pipe level"
                annotation (Placement(transformation(
                    extent={{-20,-20},{20,20}},
                    rotation=270,
                    origin={-60,100})));
              Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
                "Heat port to connect environment (negative if heat is lost to ambient)"
                annotation (Placement(transformation(extent={{-10,90},{10,110}})));

              Modelica.SIunits.Temperature T_a_inflow(start=T_start)
                "Temperature at port_a for inflowing fluid";
              Modelica.SIunits.Temperature T_b_outflow(start=T_start)
                "Temperature at port_b for outflowing fluid";
              Modelica.SIunits.Temperature TAmb=heatPort.T "Environment temperature";

            protected
              parameter Medium.ThermodynamicState sta_default=Medium.setState_pTX(
                  T=Medium.T_default,
                  p=Medium.p_default,
                  X=Medium.X_default) "Default medium state";
              parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
                  Medium.specificHeatCapacityCp(state=sta_default)
                "Heat capacity of medium";

            equation
              dp = 0;

              port_a.h_outflow = inStream(port_b.h_outflow);

              port_b.h_outflow =Medium.specificEnthalpy(
                Medium.setState_pTX(
                  port_a.p,
                  T_b_outflow,
                  port_b.Xi_outflow)) "Calculate enthalpy of output state";

                T_a_inflow = Medium.temperature(
                  Medium.setState_phX(
                    port_a.p,
                    inStream(port_a.h_outflow),
                    port_b.Xi_outflow));

              // Heat losses
              T_b_outflow = TAmb + (T_a_inflow - TAmb)*Modelica.Math.exp(-tau/tau_char);

              heatPort.Q_flow = -Buildings.Utilities.Math.Functions.spliceFunction(
                pos=(T_a_inflow - T_b_outflow)*cp_default,
                neg=0,
                x=port_a.m_flow,
                deltax=m_flow_nominal/1000)*port_a.m_flow;

              annotation (
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                    graphics={
                    Rectangle(
                      extent={{-80,80},{80,-68}},
                      lineColor={255,255,255},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),
                    Polygon(
                      points={{-52,2},{42,2},{42,8},{66,0},{42,-8},{42,-2},{-52,-2},{-52,2}},
                      lineColor={0,128,255},
                      fillPattern=FillPattern.Solid,
                      fillColor={170,213,255}),
                    Polygon(
                      points={{0,60},{38,2},{20,2},{20,-46},{-18,-46},{-18,2},{-36,2},{0,60}},
                      lineColor={0,0,0},
                      fillColor={238,46,47},
                      fillPattern=FillPattern.Solid)}),
                Documentation(info="<html>
<p>
Component that calculates the heat losses at the end of a plug flow pipe
when the flow goes in the design direction.
</p>
<h4>Main equations</h4>
<p>
The governing equations are
</p>
<p align=\"center\" style=\"font-style:italic;\">
T<sub>out</sub> = T<sub>b</sub> + (T<sub>in</sub> - T<sub>b</sub>)
exp((t<sub>out</sub> - t<sub>in</sub>)/tau<sub>char</sub>)
</p>
<p>
with
</p>
<p align=\"center\" style=\"font-style:italic;\">
tau<sub>char</sub> = R C
</p>
<h4>Assumptions and limitations</h4>
<p>
This model is based on the following assumptions:
</p>
<ul>
<li>The water temperature is uniform in the cross section.</li>
<li>There is no axial heat transfer in the water or surrounding.</li>
<li>The boundary temperature along the pipe is uniform.</li>
<li>Heat losses are steady-state.</li>
</ul>
<h4>Implementation</h4>
<p>
Heat losses are only considered in design flow direction.
For heat loss consideration in both directions, use one of these models at
both ends of a
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlow\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlow</a> model.
The outlet temperature is calculated as in the equation above,
using the inlet temperature at <code>port_a</code> and the instantaneous
time delay and boundary temperature.
The boundary temperature can be either the air temperature
or the undisturbed ground temperature, depending on the definition of the
thermal resistance <i>R</i>.
</p>
<p>
This component requires the delay time and the instantaneous ambient temperature
as an input.
This component is to be used in single pipes or in more advanced configurations
where no influence from other pipes is considered.</p>
</html>",   revisions="<html>
<ul>
<li>
December 6, 2017, by Michael Wetter:<br/>
Reformulated call to medium function.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/869\">
issue 869</a>.
</li>
<li>
October 20, 2017, by Michael Wetter:<br/>
Revised implementation to avoid graphical and textual modeling.
Revised variable names and documentation to follow guidelines.
</li>
<li>
November 6, 2015 by Bram van der Heijde:<br/>
Make time delay input instead of calculation inside this model.
</li>
<li>
September, 2015 by Marcus Fuchs:<br/>
First implementation.</li>
</ul>
</html>"),      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                        100}})));
            end PlugFlowHeatLoss;

            model PlugFlowTransportDelay "Delay time for given normalized velocity"

              parameter Modelica.SIunits.Length length "Pipe length";
              parameter Modelica.SIunits.Length dh
                "Hydraulic diameter (assuming a round cross section area)";
              parameter Modelica.SIunits.Density rho "Standard density of fluid";
              parameter Boolean initDelay=false
                "Initialize delay for a constant m_flow_start if true, otherwise start from 0"
                annotation (Dialog(group="Initialization"));
              parameter Modelica.SIunits.MassFlowRate m_flow_start=0
                "Initialization of mass flow rate to calculate initial time delay"
                annotation (Dialog(group="Initialization", enable=initDelay));

              parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
                "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

              final parameter Modelica.SIunits.Time t_in_start=
                if initDelay and (abs(m_flow_start) > 1E-10*m_flow_nominal)
                  then min(length/m_flow_start*(rho*dh^2/4*Modelica.Constants.pi), 0) else 0
                "Initial value of input time at inlet";
              final parameter Modelica.SIunits.Time t_out_start=
                if initDelay and (abs(m_flow_start) > 1E-10*m_flow_nominal)
                 then min(-length/m_flow_start*(rho*dh^2/4*Modelica.Constants.pi), 0) else 0
                "Initial value of input time at outlet";

              Modelica.SIunits.Time time_out_rev "Reverse flow direction output time";
              Modelica.SIunits.Time time_out_des "Design flow direction output time";

              Real x(start=0) "Spatial coordinate for spatialDistribution operator";
              Modelica.SIunits.Frequency u "Normalized fluid velocity (1/s)";

              Modelica.Blocks.Interfaces.RealInput m_flow "Mass flow of fluid" annotation (
                  Placement(transformation(extent={{-140,-20},{-100,20}}),
                    iconTransformation(extent={{-140,-20},{-100,20}})));

              Modelica.Blocks.Interfaces.RealOutput tau
                "Time delay for design flow direction"
                annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
              Modelica.Blocks.Interfaces.RealOutput tauRev "Time delay for reverse flow"
                annotation (Placement(transformation(extent={{100,30},{120,50}})));

            protected
              parameter Modelica.SIunits.Time t0(fixed = false) "Start time of the simulation";

            initial equation
              x = 0;
              t0 = time;

            equation
              u = m_flow/(rho*(dh^2)/4*Modelica.Constants.pi)/length;

              der(x) = u;
              (time_out_rev, time_out_des) = spatialDistribution(
                time,
                time,
                x,
                u >= 0,
                initialPoints = {0.0, 1.0},
                initialValues = {t0 + t_in_start, t0 + t_out_start});

              tau = time - time_out_des;
              tauRev = time - time_out_rev;

              annotation (
                Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                        100}})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                    graphics={
                    Rectangle(
                      extent={{-100,-100},{100,100}},
                      lineColor={0,0,127},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),
                    Line(
                      points={{-92,0},{-80.7,34.2},{-73.5,53.1},{-67.1,66.4},{-61.4,74.6},{-55.8,
                          79.1},{-50.2,79.8},{-44.6,76.6},{-38.9,69.7},{-33.3,59.4},{-26.9,44.1},
                          {-18.83,21.2},{-1.9,-30.8},{5.3,-50.2},{11.7,-64.2},{17.3,-73.1},{
                          23,-78.4},{28.6,-80},{34.2,-77.6},{39.9,-71.5},{45.5,-61.9},{51.9,
                          -47.2},{60,-24.8},{68,0}},
                      color={0,0,127},
                      smooth=Smooth.Bezier),
                    Line(points={{-64,0},{-52.7,34.2},{-45.5,53.1},{-39.1,66.4},{-33.4,74.6},
                          {-27.8,79.1},{-22.2,79.8},{-16.6,76.6},{-10.9,69.7},{-5.3,59.4},{1.1,
                          44.1},{9.17,21.2},{26.1,-30.8},{33.3,-50.2},{39.7,-64.2},{45.3,-73.1},
                          {51,-78.4},{56.6,-80},{62.2,-77.6},{67.9,-71.5},{73.5,-61.9},{79.9,
                          -47.2},{88,-24.8},{96,0}}, smooth=Smooth.Bezier),
                    Text(
                      extent={{20,100},{82,30}},
                      lineColor={0,0,255},
                      textString="PDE"),
                    Text(
                      extent={{-82,-30},{-20,-100}},
                      lineColor={0,0,255},
                      textString="tau"),
                    Text(
                      extent={{-100,140},{100,100}},
                      lineColor={0,0,255},
                      textString="%name")}),
                Documentation(info="<html>
<p>
Calculates time delay at both sides of the pipe as the difference between the
current simulation time and the inlet time of the fluid at both ends of the pipe.
</p>
<h4>Main equation</h4>
<p align=\"center\">
<i>&part;z(x,t)/&part;t + v(t) &part;z(x,t)/&part;x = 0,</i>
</p>
<p>
where <i>z(x,t)</i> is the spatial distribution as a function of time of any
property <i>z</i> of the fluid. For the inlet time propagation, <i>z</i> will
be replaced by the inlet time of the fluid <i>t<sub>in</sub></i>.
</p>
<h4>Implementation</h4>
<p>
The inlet time is approached as a fluid property and its propagation follows
the one-dimensional wave equation, implemented using the spatialDistribution
function. This components requires the mass flow through the pipe and the pipe
dimensions in order to derive information about the fluid propagation.
</p>
<p>
The component calculates the delay time at both in/outlet ports of the pipe
and therefore has two outlets. During forward flow, only the forward
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowTransportDelay\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowTransportDelay</a> component in
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore</a>
will be active and uses the forward output of PlugFlowTransportDelay.
During reverse, the opposite is true and only the reverse output is used.
</p>
<h4>Assumption</h4>
<p>It is assumed that no axial mixing takes place in the pipe. </p>
</html>",             revisions="<html>
<ul>
<li>
December 14, 2018, by Michael Wetter:<br/>
Corrected argument of <code>spatialDistribution</code> operator to be a parameter
expression.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1055\">#1055</a>.
<li>
September 9, 2016 by Bram van der Heijde:<br/>
Rename from PDETime_massFlowMod to PlugFlowTransportDelayMod
</li>
<li>
December 2015 by Carles Ribas Tugores:<br/>
Modification in delay calculation to fix issues.
</li>
<li>
November 6, 2015 by Bram van der Heijde:<br/>
Adapted flow parameter to mass flow rate instead of velocity.
This change should also fix the reverse and zero flow issues.
</li>
<li>
October 13, 2015 by Marcus Fuchs:<br/>
Use <code>abs()</code> of normalized velocity input in order to avoid negative
delay times.
</li>
<li>
July 2015 by Arnout Aertgeerts:<br/>
First implementation.
</li>
</ul>
</html>"));
            end PlugFlowTransportDelay;
          annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.FixedResistances\">
Buildings.Fluid.FixedResistances</a>.
</p>
</html>"));
          end BaseClasses;
          annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains component models for fixed flow resistances.
By fixed flow resistance, we mean resistances that do not change the
flow coefficient
</p>
<p align=\"center\" style=\"font-style:italic;\">
k = m &frasl;
&radic;<span style=\"text-decoration:overline;\">&Delta;P</span>.
</p>
<p>
For models of valves and air dampers, see
<a href=\"modelica://Buildings.Fluid.Actuators\">
Buildings.Fluid.Actuators</a>.
For models of flow resistances as part of the building constructions, see
<a href=\"modelica://Buildings.Airflow.Multizone\">
Buildings.Airflow.Multizone</a>.
</p>
<p>
The model
<a href=\"modelica://Buildings.Fluid.FixedResistances.PressureDrop\">
Buildings.Fluid.FixedResistances.PressureDrop</a>
is a fixed flow resistance that takes as parameter a nominal flow rate and a nominal pressure drop. The actual resistance is scaled using the above equation.
</p>
<p>
The model
<a href=\"modelica://Buildings.Fluid.FixedResistances.HydraulicDiameter\">
Buildings.Fluid.FixedResistances.HydraulicDiameter</a>
is a fixed flow resistance that takes as parameter a nominal flow rate and
a hydraulic diameter. The actual resistance is scaled using the above equation.
</p>
<p>
The model
<a href=\"modelica://Buildings.Fluid.FixedResistances.LosslessPipe\">
Buildings.Fluid.FixedResistances.LosslessPipe</a>
is an ideal pipe segment with no pressure drop. It is primarily used
in models in which the above pressure drop model need to be replaced by a model with no pressure drop.
</p>
<p>
The model
<a href=\"modelica://Buildings.Fluid.FixedResistances.Junction\">
Buildings.Fluid.FixedResistances.Junction</a>
can be used to model flow splitters or flow merges.
</p>
</html>"));
        end FixedResistances;

        package MixingVolumes "Package with mixing volumes"
          extends Modelica.Icons.VariantsPackage;

          model MixingVolume
            "Mixing volume with inlet and outlet ports (flow reversal is allowed)"
            extends Buildings.Fluid.MixingVolumes.BaseClasses.PartialMixingVolume(
              final initialize_p=not Medium.singleState,
              steBal(final use_C_flow=use_C_flow),
              dynBal(final use_C_flow=use_C_flow));

            parameter Boolean use_C_flow = false
              "Set to true to enable input connector for trace substance"
              annotation(Evaluate=true, Dialog(tab="Advanced"));

            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(
              T(start=T_start)) "Heat port for heat exchange with the control volume"
              annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

            Modelica.Blocks.Interfaces.RealInput[Medium.nC] C_flow if use_C_flow
              "Trace substance mass flow rate added to the medium"
              annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));

          equation
            connect(heaFloSen.port_a, heatPort)
              annotation (Line(points={{-90,0},{-96,0},{-100,0}}, color={191,0,0}));
            connect(C_flow, steBal.C_flow) annotation (Line(points={{-120,-60},{-80,-60},
                    {12,-60},{12,6},{18,6}}, color={0,0,127}));
            connect(C_flow, dynBal.C_flow) annotation (Line(points={{-120,-60},{-52,-60},
                    {52,-60},{52,6},{58,6}}, color={0,0,127}));
            annotation (
          defaultComponentName="vol",
          Documentation(info="<html>
<p>
This model represents an instantaneously mixed volume.
Potential and kinetic energy at the port are neglected,
and there is no pressure drop at the ports.
The volume can exchange heat through its <code>heatPort</code>.
</p>
<p>
The volume can be parameterized as a steady-state model or as
dynamic model.</p>
<p>
To increase the numerical robustness of the model, the constant
<code>prescribedHeatFlowRate</code> can be set by the user.
This constant only has an effect if the model has exactly two fluid ports connected,
and if it is used as a steady-state model.
Use the following settings:
</p>
<ul>
<li>Set <code>prescribedHeatFlowRate=true</code> if the <i>only</i> means of heat transfer
at the <code>heatPort</code> is a prescribed heat flow rate that
is <i>not</i> a function of the temperature difference
between the medium and an ambient temperature. Examples include an ideal electrical heater,
a pump that rejects heat into the fluid stream, or a chiller that removes heat based on a performance curve.
If the <code>heatPort</code> is not connected, then set <code>prescribedHeatFlowRate=true</code> as
in this case, <code>heatPort.Q_flow=0</code>.
</li>
<li>Set <code>prescribedHeatFlowRate=false</code> if there is heat flow at the <code>heatPort</code>
computed as <i>K * (T-heatPort.T)</i>, for some temperature <i>T</i> and some conductance <i>K</i>,
which may itself be a function of temperature or mass flow rate.<br/>
If there is a combination of <i>K * (T-heatPort.T)</i> and a prescribed heat flow rate,
for example a solar collector that dissipates heat to the ambient and receives heat from
the solar radiation, then set <code>prescribedHeatFlowRate=false</code>.
</li>
</ul>
<h4>Options</h4>
<p>
The parameter <code>mSenFac</code> can be used to increase the thermal mass of this model
without increasing its volume. This way, species concentrations are still calculated
correctly even though the thermal mass increases. The additional thermal mass is calculated
based on the density and the value of the function <code>HeatCapacityCp</code>
of the medium state <code>state_default</code>. <br/>
This parameter can for instance be useful in a pipe model when the developer wants to
lump the pipe thermal mass to the fluid volume. By default <code>mSenFac = 1</code>, hence
the mass is unchanged. For higher values of <code>mSenFac</code>, the mass will be scaled proportionally.
</p>
<p>
Set the parameter <code>use_C_flow = true</code> to enable an input connector for the trace substance flow rate.
This allows to directly add or subtract trace substances such as
CO2 to the volume.
See
<a href=\"modelica://Buildings.Fluid.Sensors.Examples.PPM\">Buildings.Fluid.Sensors.Examples.PPM</a>
for an example.
</p>
<h4>Implementation</h4>
<p>
If the model is operated in steady-state and has two fluid ports connected,
then the same energy and mass balance implementation is used as in
steady-state component models, i.e., the use of <code>actualStream</code>
is not used for the properties at the port.
</p>
<p>
The implementation of these balance equations is done in the instances
<code>dynBal</code> for the dynamic balance and <code>steBal</code>
for the steady-state balance. Both models use the same input variables:
</p>
<ul>
<li>
The variable <code>Q_flow</code> is used to add sensible <i>and</i> latent heat to the fluid.
For example, <code>Q_flow</code> participates in the steady-state energy balance<pre>
    port_b.h_outflow = inStream(port_a.h_outflow) + Q_flow * m_flowInv;
</pre>
where <code>m_flowInv</code> approximates the expression <code>1/m_flow</code>.
</li>
<li>
The variable <code>mXi_flow</code> is used to add a species mass flow rate to the fluid.
</li>
</ul>
<p>
For the rationale of selecting different energy and mass balances, and for the
use of <code>prescribedHeatFlowRate</code>, see the documentation of
<a href=\"modelica://Buildings.Fluid.MixingVolumes.BaseClasses.PartialMixingVolume\">
Buildings.Fluid.MixingVolumes.BaseClasses.PartialMixingVolume</a>.
</p>
<p>
For simple models that uses this model, see
<a href=\"modelica://Buildings.Fluid.HeatExchangers.HeaterCooler_u\">
Buildings.Fluid.HeatExchangers.HeaterCooler_u</a> and
<a href=\"modelica://Buildings.Fluid.Humidifiers.Humidifier_u\">
Buildings.Fluid.Humidifiers.Humidifier_u</a>.
</p>

</html>",           revisions="<html>
<ul>
<li>
October 19, 2017, by Michael Wetter:<br/>
Set <code>initialize_p</code> to <code>final</code> so that it does not
appear as a user-selectable parameter. This is done because
<code>initialize_p</code> has been changed from a <code>constant</code>
to a <code>parameter</code> for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1013\">Buildings, issue 1013</a>.
</li>
<li>
April 11, 2017, by Michael Wetter:<br/>
Changed comment of heat port, as this needs to be the total heat flow
rate in order to be able to use this model for modeling steam humidifiers
and adiabatic humidifiers.<br/>
Removed blocks <code>QSen_flow</code> and
<code>QLat_flow</code>.<br/>
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/704\">Buildings #704</a>.
</li>
<li>
April 11, 2016 by Michael Wetter:<br/>
Corrected wrong hyperlink in documentation for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/450\">issue 450</a>.
</li>
<li>
January 19, 2016, by Michael Wetter:<br/>
Updated documentation due to the addition of an input for trace substance
in the mixing volume.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/372\">
issue 372</a>.
</li>
<li>
January 17, 2016, by Michael Wetter:<br/>
Removed <code>protected</code> block <code>masExc</code> as
this revision introduces a conditional connector for the
moisture flow rate in the energy and mass balance models.
This change was done to use the same modeling concept for the
moisture input as is used for the trace substance input.
</li>
<li>
December 2, 2015, by Filip Jorissen:<br/>
Changed code for handling trace substance insertions using input <code>C_flow</code>.
</li>
<li>
May 1, 2015 by Michael Wetter<br/>
Set <code>final</code> keyword for <code>masExc(final k=0)</code>.
This addresses
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/230\">
issue 230</a>.
</li>
<li>
February 11, 2014 by Michael Wetter:<br/>
Redesigned implementation of latent and sensible heat flow rates
as port of the correction of issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/197\">#197</a>.
</li>
<li>
February 7, 2012 by Michael Wetter:<br/>
Revised base classes for conservation equations in <code>Buildings.Fluid.Interfaces</code>.
</li>
<li>
September 17, 2011 by Michael Wetter:<br/>
Removed instance <code>medium</code> as this is already used in <code>dynBal</code>.
Removing the base properties led to 30% faster computing time for a solar thermal system
that contains many fluid volumes.
</li>
<li>
September 13, 2011 by Michael Wetter:<br/>
Changed in declaration of <code>medium</code> the parameter assignment
<code>preferredMediumStates=true</code> to
<code>preferredMediumStates= not (energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState)</code>.
Otherwise, for a steady-state model, Dymola 2012 may differentiate the model to obtain <code>T</code>
as a state. See ticket Dynasim #13596.
</li>
<li>
July 26, 2011 by Michael Wetter:<br/>
Revised model to use new declarations from
<a href=\"Buildings.Fluid.Interfaces.LumpedVolumeDeclarations\">
Buildings.Fluid.Interfaces.LumpedVolumeDeclarations</a>.
</li>
<li>
July 14, 2011 by Michael Wetter:<br/>
Added start values for mass and internal energy of dynamic balance
model.
</li>
<li>
May 25, 2011 by Michael Wetter:<br/>
<ul>
<li>
Changed implementation of balance equation. The new implementation uses a different model if
exactly two fluid ports are connected, and in addition, the model is used as a steady-state
component. For this model configuration, the same balance equations are used as were used
for steady-state component models, i.e., instead of <code>actualStream(...)</code>, the
<code>inStream(...)</code> formulation is used.
This changed required the introduction of a new parameter <code>m_flow_nominal</code> which
is used for smoothing in the steady-state balance equations of the model with two fluid ports.
</li>
<li>
Another revision was the removal of the parameter <code>use_HeatTransfer</code> as there is
no noticeable overhead in always having the <code>heatPort</code> connector present.
</li>
</ul>
</li>
<li>
July 30, 2010 by Michael Wetter:<br/>
Added nominal value for <code>mC</code> to avoid wrong trajectory
when concentration is around 1E-7.
See also <a href=\"https://trac.modelica.org/Modelica/ticket/393\">
https://trac.modelica.org/Modelica/ticket/393</a>.
</li>
<li>
February 7, 2010 by Michael Wetter:<br/>
Simplified model and its base classes by removing the port data
and the vessel area.
Eliminated the base class <code>PartialLumpedVessel</code>.
</li>
<li>
October 12, 2009 by Michael Wetter:<br/>
Changed base class to
<a href=\"modelica://Buildings.Fluid.MixingVolumes.BaseClasses.ClosedVolume\">
Buildings.Fluid.MixingVolumes.BaseClasses.ClosedVolume</a>.
</li>
</ul>
</html>"),    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                      100}}),
                graphics={
                Text(
                    extent={{-152,100},{148,140}},
                    textString="%name",
                    lineColor={0,0,255})}));
          end MixingVolume;

          package BaseClasses "Package with base classes for Buildings.Fluid.MixingVolumes"
            extends Modelica.Icons.BasesPackage;

            model PartialMixingVolume
              "Partial mixing volume with inlet and outlet ports (flow reversal is allowed)"

              extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
              parameter Boolean initialize_p = not Medium.singleState
                "= true to set up initial equations for pressure"
                annotation(HideResult=true, Evaluate=true, Dialog(tab="Advanced"));

              // We set prescribedHeatFlowRate=false so that the
              // volume works without the user having to set this advanced parameter,
              // but to get high robustness, a user can set it to the appropriate value
              // as described in the info section.
              constant Boolean prescribedHeatFlowRate = false
                "Set to true if the model has a prescribed heat flow at its heatPort. If the heat flow rate at the heatPort is only based on temperature difference, then set to false";

              constant Boolean simplify_mWat_flow = true
                "Set to true to cause port_a.m_flow + port_b.m_flow = 0 even if mWat_flow is non-zero";

              parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
                "Nominal mass flow rate"
                annotation(Dialog(group = "Nominal condition"));
              // Port definitions
              parameter Integer nPorts=0 "Number of ports"
                annotation(Evaluate=true, Dialog(connectorSizing=true, tab="General",group="Ports"));
              parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
                "Small mass flow rate for regularization of zero flow"
                annotation(Dialog(tab = "Advanced"));
              parameter Boolean allowFlowReversal = true
                "= false to simplify equations, assuming, but not enforcing, no flow reversal. Used only if model has two ports."
                annotation(Dialog(tab="Assumptions"), Evaluate=true);
              parameter Modelica.SIunits.Volume V "Volume";
              Modelica.Fluid.Vessels.BaseClasses.VesselFluidPorts_b ports[nPorts](
                  redeclare each package Medium = Medium) "Fluid inlets and outlets"
                annotation (Placement(transformation(extent={{-40,-10},{40,10}},
                  origin={0,-100})));

              Medium.Temperature T = Medium.temperature_phX(p=p, h=hOut_internal, X=cat(1,Xi,{1-sum(Xi)}))
                "Temperature of the fluid";
              Modelica.Blocks.Interfaces.RealOutput U(unit="J")
                "Internal energy of the component";
              Modelica.SIunits.Pressure p = if nPorts > 0 then ports[1].p else p_start
                "Pressure of the fluid";
              Modelica.Blocks.Interfaces.RealOutput m(unit="kg") "Mass of the component";
              Modelica.SIunits.MassFraction Xi[Medium.nXi] = XiOut_internal
                "Species concentration of the fluid";
              Modelica.Blocks.Interfaces.RealOutput mXi[Medium.nXi](each unit="kg")
                "Species mass of the component";
              Medium.ExtraProperty C[Medium.nC](nominal=C_nominal) = COut_internal
                "Trace substance mixture content";
              Modelica.Blocks.Interfaces.RealOutput mC[Medium.nC](each unit="kg")
                "Trace substance mass of the component";

            protected
              Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation steBal(
                final simplify_mWat_flow=simplify_mWat_flow,
                redeclare final package Medium = Medium,
                final m_flow_nominal=m_flow_nominal,
                final allowFlowReversal=allowFlowReversal,
                final m_flow_small=m_flow_small,
                final prescribedHeatFlowRate=prescribedHeatFlowRate) if useSteadyStateTwoPort "Model for steady-state balance if nPorts=2" annotation (Placement(transformation(extent={{20,0},{40,20}})));
              Buildings.Fluid.Interfaces.ConservationEquation dynBal(
                final simplify_mWat_flow=simplify_mWat_flow,
                redeclare final package Medium = Medium,
                final energyDynamics=energyDynamics,
                final massDynamics=massDynamics,
                final p_start=p_start,
                final T_start=T_start,
                final X_start=X_start,
                final C_start=C_start,
                final C_nominal=C_nominal,
                final fluidVolume=V,
                final initialize_p=initialize_p,
                m(start=V*rho_start),
                nPorts=nPorts,
                final mSenFac=mSenFac) if not useSteadyStateTwoPort "Model for dynamic energy balance" annotation (Placement(transformation(extent={{60,0},{80,20}})));

              // Density at start values, used to compute initial values and start guesses
              parameter Modelica.SIunits.Density rho_start=Medium.density(
               state=state_start) "Density, used to compute start and guess values";
              final parameter Medium.ThermodynamicState state_default = Medium.setState_pTX(
                  T=Medium.T_default,
                  p=Medium.p_default,
                  X=Medium.X_default[1:Medium.nXi]) "Medium state at default values";
              // Density at medium default values, used to compute the size of control volumes
              final parameter Modelica.SIunits.Density rho_default=Medium.density(
                state=state_default) "Density, used to compute fluid mass";
              final parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(
                  T=T_start,
                  p=p_start,
                  X=X_start[1:Medium.nXi]) "Medium state at start values";
              // See info section for why prescribedHeatFlowRate is used here.
              // The condition below may only be changed if StaticTwoPortConservationEquation
              // contains a correct solution for all foreseeable parameters/inputs.
              // See Buildings, issue 282 for a discussion.
              final parameter Boolean useSteadyStateTwoPort=(nPorts == 2) and
                  (prescribedHeatFlowRate or (not allowFlowReversal)) and (
                  energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) and (
                  massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) and (
                  substanceDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) and (
                  traceDynamics == Modelica.Fluid.Types.Dynamics.SteadyState)
                "Flag, true if the model has two ports only and uses a steady state balance"
                annotation (Evaluate=true);
              // Outputs that are needed to assign the medium properties
              Modelica.Blocks.Interfaces.RealOutput hOut_internal(unit="J/kg")
                "Internal connector for leaving temperature of the component";
              Modelica.Blocks.Interfaces.RealOutput XiOut_internal[Medium.nXi](each unit="1")
                "Internal connector for leaving species concentration of the component";
              Modelica.Blocks.Interfaces.RealOutput COut_internal[Medium.nC](each unit="1")
                "Internal connector for leaving trace substances of the component";

              Buildings.HeatTransfer.Sources.PrescribedTemperature preTem "Port temperature" annotation (Placement(transformation(extent={{-40,-10},{-60,10}})));
              Modelica.Blocks.Sources.RealExpression portT(y=T) "Port temperature"
                annotation (Placement(transformation(extent={{-10,-10},{-30,10}})));
              Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heaFloSen
                "Heat flow sensor"
                annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
            equation
              ///////////////////////////////////////////////////////////////////////////
              // asserts
              if not allowFlowReversal then
                assert(ports[1].m_flow > -m_flow_small,
              "In " + getInstanceName() + ": Model has flow reversal,
  but the parameter allowFlowReversal is set to false.
  m_flow_small    = "             + String(m_flow_small) + "
  ports[1].m_flow = "             + String(ports[1].m_flow) + "
");           end if;
              // Actual definition of port variables.
              //
              // If the model computes the energy and mass balances as steady-state,
              // and if it has only two ports,
              // then we use the same base class as for all other steady state models.
              if useSteadyStateTwoPort then
              connect(steBal.port_a, ports[1]) annotation (Line(
                  points={{20,10},{10,10},{10,-20},{0,-20},{0,-20},{0,-100}},
                  color={0,127,255}));

              connect(steBal.port_b, ports[2]) annotation (Line(
                  points={{40,10},{46,10},{46,-20},{0,-20},{0,-100}},
                  color={0,127,255}));
                U=0;
                mXi=zeros(Medium.nXi);
                m=0;
                mC=zeros(Medium.nC);
                connect(hOut_internal,  steBal.hOut);
                connect(XiOut_internal, steBal.XiOut);
                connect(COut_internal,  steBal.COut);
              else
                  connect(dynBal.ports, ports) annotation (Line(
                  points={{70,0},{70,-80},{62,-80},{2.22045e-15,-80},{2.22045e-15,-90},{2.22045e-15,
                        -100}},
                  color={0,127,255}));
                connect(U,dynBal.UOut);
                connect(mXi,dynBal.mXiOut);
                connect(m,dynBal.mOut);
                connect(mC,dynBal.mCOut);
                connect(hOut_internal,  dynBal.hOut);
                connect(XiOut_internal, dynBal.XiOut);
                connect(COut_internal,  dynBal.COut);
              end if;

              connect(portT.y, preTem.T)
                annotation (Line(points={{-31,0},{-38,0}},   color={0,0,127}));
              connect(heaFloSen.port_b, preTem.port)
                annotation (Line(points={{-70,0},{-65,0},{-60,0}},    color={191,0,0}));
              connect(heaFloSen.Q_flow, steBal.Q_flow) annotation (Line(points={{-80,-10},{
                      -80,-16},{6,-16},{6,18},{18,18}},
                                                 color={0,0,127}));
              connect(heaFloSen.Q_flow, dynBal.Q_flow) annotation (Line(points={{-80,-10},{
                      -80,-10},{-80,-16},{6,-16},{6,24},{50,24},{50,16},{58,16}},
                                                                           color={0,0,127}));
              annotation (
            defaultComponentName="vol",
            Documentation(info="<html>
<p>
This is a partial model of an instantaneously mixed volume.
It is used as the base class for all fluid volumes of the package
<a href=\"modelica://Buildings.Fluid.MixingVolumes\">
Buildings.Fluid.MixingVolumes</a>.
</p>


<h4>Typical use and important parameters</h4>
<p>
Set the constant <code>sensibleOnly=true</code> if the model that extends
or instantiates this model sets <code>mWat_flow = 0</code>.
</p>
<p>
Set the constant <code>simplify_mWat_flow = true</code> to simplify the equation
</p>
<pre>
  port_a.m_flow + port_b.m_flow = - mWat_flow;
</pre>
<p>
to
</p>
<pre>
  port_a.m_flow + port_b.m_flow = 0;
</pre>
<p>
This causes an error in the mass balance of about <i>0.5%</i>, but generally leads to
simpler equations because the pressure drop equations are then decoupled from the
mass exchange in this component.
</p>

<p>
To increase the numerical robustness of the model, the constant
<code>prescribedHeatFlowRate</code> can be set by the user.
This constant only has an effect if the model has exactly two fluid ports connected,
and if it is used as a steady-state model.
Use the following settings:
</p>
<ul>
<li>Set <code>prescribedHeatFlowRate=true</code> if the <i>only</i> means of heat transfer
at the <code>heatPort</code> is a prescribed heat flow rate that
is <i>not</i> a function of the temperature difference
between the medium and an ambient temperature. Examples include an ideal electrical heater,
a pump that rejects heat into the fluid stream, or a chiller that removes heat based on a performance curve.
If the <code>heatPort</code> is not connected, then set <code>prescribedHeatFlowRate=true</code> as
in this case, <code>heatPort.Q_flow=0</code>.
</li>
<li>Set <code>prescribedHeatFlowRate=false</code> if there is heat flow at the <code>heatPort</code>
computed as <i>K * (T-heatPort.T)</i>, for some temperature <i>T</i> and some conductance <i>K</i>,
which may itself be a function of temperature or mass flow rate.<br/>
If there is a combination of <i>K * (T-heatPort.T)</i> and a prescribed heat flow rate,
for example a solar collector that dissipates heat to the ambient and receives heat from
the solar radiation, then set <code>prescribedHeatFlowRate=false</code>.
</li>
</ul>
<p>
Set the parameter <code>use_C_flow = true</code> to enable an input connector
for the trace substance flow rate.
</p>
<h4>Implementation</h4>
<p>
If the model is (i) operated in steady-state,
(ii) has two fluid ports connected, and
(iii) <code>prescribedHeatFlowRate=true</code> or <code>allowFlowReversal=false</code>,
then the model uses
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation\">
Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation</a>
in order to use
the same energy and mass balance implementation as is used as in
steady-state component models.
In this situation, the functions <code>inStream</code> are used for the two
flow directions rather than the function
<code>actualStream</code>, which is less efficient.
However, the use of <code>inStream</code> has the disadvantage
that <code>hOut</code> has to be computed, in
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation\">
Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation</a>,
using
</p>
<pre>
if allowFlowReversal then
  hOut = Buildings.Utilities.Math.Functions.regStep(y1=port_b.h_outflow,
                                                    y2=port_a.h_outflow,
                                                    x=port_a.m_flow,
                                                    x_small=m_flow_small/1E3);
else
  hOut = port_b.h_outflow;
end if;
</pre>
<p>
Hence, for <code>allowFlowReversal=true</code>, if <code>hOut</code>
were to be used to compute the temperature that
drives heat transfer such as by conduction,
then the heat transfer would depend on upstream and the <i>downstream</i>
temperatures for small mass flow rates.
This can give wrong results. Consider for example a mass flow rate that is positive
but very close to zero. Suppose the upstream temperature is <i>20</i>&deg;C,
the downstream temperature is <i>10</i>&deg;C, and the heat port is
connected through a heat conductor to a boundary condition of <i>20</i>&deg;C.
Then, <code>hOut = (port_b.h_outflow + port_a.h_outflow)/2</code> and hence
the temperature <code>heatPort.T</code>
is <i>15</i>&deg;C. Therefore, heat is added to the component.
As the mass flow rate is by assumption very small, the fluid that leaves the component
will have a very high temperature, violating the 2nd law.
To avoid this situation, if
<code>prescribedHeatFlowRate=false</code>, then the model
<a href=\"modelica://Buildings.Fluid.Interfaces.ConservationEquation\">
Buildings.Fluid.Interfaces.ConservationEquation</a>
is used instead of
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation\">
Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation</a>.
</p>
<p>
For simple models that uses this model, see
<a href=\"modelica://Buildings.Fluid.MixingVolumes\">
Buildings.Fluid.MixingVolumes</a>.
</p>
</html>",             revisions="<html>
<ul>
<li>
February 21, 2020, by Michael Wetter:<br/>
Changed icon to display its operating state.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1294\">#1294</a>.
</li>
<li>
October 30, 2019 by Filip Jorissen:<br/>
Added <code>getInstanceName()</code> to flow
reversal check.
This if or <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1228\">
issue 1228</a>.
</li>
<li>
October 19, 2017, by Michael Wetter:<br/>
Changed initialization of pressure from a <code>constant</code> to a <code>parameter</code>.<br/>
Removed <code>partial</code> keyword as this model is not partial.<br/>
Moved <code>C_flow</code> and <code>use_C_flow</code> to child classes.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1013\">Buildings, issue 1013</a>.
</li>
<li>
April 11, 2017, by Michael Wetter:<br/>
Moved heat port to the extending classes to provide better comment.
Otherwise, the mixing volume without water input would have a comment
that says latent heat can be added at this port.<br/>
Removed blocks <code>QSen_flow</code> and
<code>QLat_flow</code>.<br/>
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/704\">Buildings #704</a>.
</li>
<li>
February 19, 2016 by Filip Jorissen:<br/>
Added outputs U, m, mXi, mC for being able to
check conservation of quantities.
This if or <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/247\">
issue 247</a>.
</li>
<li>
January 22, 2016 by Michael Wetter:<br/>
Updated model to use the new parameter <code>use_mWat_flow</code>
rather than <code>sensibleOnly</code>.
</li>
<li>
January 17, 2016, by Michael Wetter:<br/>
Removed <code>protected</code> block <code>masExc</code> as
this revision introduces a conditional connector for the
moisture flow rate in the energy and mass balance models.
This change was done to use the same modeling concept for the
moisture input as is used for the trace substance input.
</li>
<li>
December 2, 2015, by Filip Jorissen:<br/>
Added conditional input <code>C_flow</code> for
handling trace substance insertions.
</li>
<li>
July 17, 2015, by Michael Wetter:<br/>
Added constant <code>simplify_mWat_flow</code> to remove dependencies of the pressure drop
calculation on the moisture balance.
</li>
<li>
July 1, 2015, by Filip Jorissen:<br/>
Set <code>prescribedHeatFlowRate=prescribedHeatflowRate</code> for
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation\">
Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation</a>.
This results in equations that are solved more easily.
See
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/282\">
issue 282</a> for a discussion.
</li>
<li>
June 9, 2015 by Michael Wetter:<br/>
Set start value for <code>heatPort.T</code> and changed
type of <code>T</code> to <code>Medium.Temperature</code> rather than
<code>Modelica.SIunits.Temperature</code>
to avoid an
error because of conflicting start values if
<code>Buildings.Fluid.Chillers.Carnot_y</code>
is translated using pedantic mode in Dymola 2016.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/426\">#426</a>.
</li>
<li>
June 5, 2015, by Michael Wetter:<br/>
Moved assignment of <code>dynBal.U.start</code>
from instance <code>dynBal</code> to the actual model implementation.
This is required for a pedantic model check in Dymola 2016.
It addresses
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/266\">
issue 266</a>.
</li>
<li>
May 6, 2015, by Michael Wetter:<br/>
Improved documentation and changed the test
<pre>
 final parameter Boolean useSteadyStateTwoPort=(nPorts == 2) and
 prescribedHeatFlowRate and ...
</pre>
to
<pre>
 final parameter Boolean useSteadyStateTwoPort=(nPorts == 2) and
 (prescribedHeatFlowRate or (not allowFlowReversal)) and ...
</pre>
The reason is that if there is no flow reversal, then
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation\">
Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation</a>
computes <code>hOut =  port_b.h_outflow;</code>, and hence
it is correct to use <code>hOut</code> to compute
temperature-driven heat flow, such as by conduction or convection.
See also the model documentation.<br/>
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/412\">
#412</a>.
</li>
<li>
February 5, 2015, by Michael Wetter:<br/>
Changed <code>initalize_p</code> from a <code>parameter</code> to a
<code>constant</code>. This is only required in finite volume models
of heat exchangers (to avoid consistent but redundant initial conditions)
and hence it should be set as a <code>constant</code>.
</li>
<li>
October 29, 2014, by Michael Wetter:<br/>
Made assignment of <code>mFactor</code> final, and changed computation of
density to use default medium states as are also used to compute the
specific heat capacity.
</li>
<li>
October 21, 2014, by Filip Jorissen:<br/>
Added parameter <code>mFactor</code> to increase the thermal capacity.
</li>
<li>
July 3, 2014, by Michael Wetter:<br/>
Added parameter <code>initialize_p</code>. This is required
to enable the coil models to initialize the pressure in the first
volume, but not in the downstream volumes. Otherwise,
the initial equations will be overdetermined, but consistent.
This change was done to avoid a long information message that appears
when translating models.
</li>
<li>
May 29, 2014, by Michael Wetter:<br/>
Removed undesirable annotation <code>Evaluate=true</code>.
</li>
<li>
February 11, 2014 by Michael Wetter:<br/>
Removed <code>Q_flow</code> and added <code>QSen_flow</code>.
This was done to clarify what is sensible and total heat flow rate
as part of the correction of issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/197\">#197</a>.
</li>
<li>
October 8, 2013 by Michael Wetter:<br/>
Removed propagation of <code>show_V_flow</code>
to instance <code>steBal</code> as it has no longer this parameter.
</li>
<li>
September 13, 2013 by Michael Wetter:<br/>
Renamed <code>rho_nominal</code> to <code>rho_start</code>
because this quantity is computed using start values and not
nominal values.
</li>
<li>
April 18, 2013 by Michael Wetter:<br/>
Removed the check of multiple connections to the same element
of a fluid port, as this check required the use of the deprecated
<code>cardinality</code> function.
</li>
<li>
February 7, 2012 by Michael Wetter:<br/>
Revised base classes for conservation equations in <code>Buildings.Fluid.Interfaces</code>.
</li>
<li>
September 17, 2011 by Michael Wetter:<br/>
Removed instance <code>medium</code> as this is already used in <code>dynBal</code>.
Removing the base properties led to 30% faster computing time for a solar thermal system
that contains many fluid volumes.
</li>
<li>
September 13, 2011 by Michael Wetter:<br/>
Changed in declaration of <code>medium</code> the parameter assignment
<code>preferredMediumStates=true</code> to
<code>preferredMediumStates= not (energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState)</code>.
Otherwise, for a steady-state model, Dymola 2012 may differentiate the model to obtain <code>T</code>
as a state. See ticket Dynasim #13596.
</li>
<li>
July 26, 2011 by Michael Wetter:<br/>
Revised model to use new declarations from
<a href=\"Buildings.Fluid.Interfaces.LumpedVolumeDeclarations\">
Buildings.Fluid.Interfaces.LumpedVolumeDeclarations</a>.
</li>
<li>
July 14, 2011 by Michael Wetter:<br/>
Added start values for mass and internal energy of dynamic balance
model.
</li>
<li>
May 25, 2011 by Michael Wetter:<br/>
<ul>
<li>
Changed implementation of balance equation. The new implementation uses a different model if
exactly two fluid ports are connected, and in addition, the model is used as a steady-state
component. For this model configuration, the same balance equations are used as were used
for steady-state component models, i.e., instead of <code>actualStream(...)</code>, the
<code>inStream(...)</code> formulation is used.
This changed required the introduction of a new parameter <code>m_flow_nominal</code> which
is used for smoothing in the steady-state balance equations of the model with two fluid ports.
This implementation also simplifies the implementation of
<a href=\"modelica://Buildings.Fluid.MixingVolumes.BaseClasses.PartialMixingVolumeWaterPort\">
Buildings.Fluid.MixingVolumes.BaseClasses.PartialMixingVolumeWaterPort</a>,
which now uses the same equations as this model.
</li>
<li>
Another revision was the removal of the parameter <code>use_HeatTransfer</code> as there is
no noticeable overhead in always having the <code>heatPort</code> connector present.
</li>
</ul>
</li>
<li>
July 30, 2010 by Michael Wetter:<br/>
Added nominal value for <code>mC</code> to avoid wrong trajectory
when concentration is around 1E-7.
See also <a href=\"https://trac.modelica.org/Modelica/ticket/393\">
https://trac.modelica.org/Modelica/ticket/393</a>.
</li>
<li>
February 7, 2010 by Michael Wetter:<br/>
Simplified model and its base classes by removing the port data
and the vessel area.
Eliminated the base class <code>PartialLumpedVessel</code>.
</li>
<li>
October 12, 2009 by Michael Wetter:<br/>
Changed base class to
<a href=\"modelica://Buildings.Fluid.MixingVolumes.BaseClasses.ClosedVolume\">
Buildings.Fluid.MixingVolumes.BaseClasses.ClosedVolume</a>.
</li>
</ul>
</html>"),      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                        100}}), graphics={
                   Text(
                      extent={{-60,-26},{56,-58}},
                      lineColor={255,255,255},
                      textString="V=%V"),
                    Text(
                      extent={{-152,100},{148,140}},
                      textString="%name",
                      lineColor={0,0,255}),
                   Ellipse(
                      extent={{-100,98},{100,-102}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.Sphere,
                      fillColor=DynamicSelect({170,213,255}, min(1, max(0, (1-(T-273.15)/50)))*{28,108,200}+min(1, max(0, (T-273.15)/50))*{255,0,0})),
                    Text(
                      extent={{62,28},{-58,-22}},
                      lineColor={255,255,255},
                      textString=DynamicSelect("", String(T-273.15, format=".1f")))}));
            end PartialMixingVolume;
          annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.MixingVolumes\">Buildings.Fluid.MixingVolumes</a>.
</p>
</html>"));
          end BaseClasses;
        annotation (Documentation(info="<html>
<p>
This package contains models for completely mixed volumes.
</p>
<p>
For most situations, the model
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolume\">
Buildings.Fluid.MixingVolumes.MixingVolume</a> should be used.
The other models are only of interest if water should be added to
or subtracted from the fluid volume, such as in a
coil with water vapor condensation.
</p>
</html>"));
        end MixingVolumes;

        package Sensors "Package with sensor models"
          extends Modelica.Icons.SensorsPackage;

          model MassFlowRate "Ideal sensor for mass flow rate"
          extends Buildings.Fluid.Sensors.BaseClasses.PartialFlowSensor(final m_flow_nominal=0, final m_flow_small=0);
          extends Modelica.Icons.RotationalSensor;
          Modelica.Blocks.Interfaces.RealOutput m_flow(quantity="MassFlowRate",
                                                       final unit="kg/s")
            "Mass flow rate from port_a to port_b" annotation (Placement(
                transformation(
                origin={0,110},
                extent={{10,-10},{-10,10}},
                rotation=270)));

          equation
          m_flow = port_a.m_flow;
            annotation (
              defaultComponentName="senMasFlo",
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                graphics={
                Line(points={{70,0},{100,0}}, color={0,128,255}),
                Text(
                  extent={{162,120},{2,90}},
                  lineColor={0,0,0},
                  textString="m_flow"),
                Line(points={{0,100},{0,70}}, color={0,0,127}),
                Line(points={{-100,0},{-70,0}}, color={0,128,255}),
                Text(
                  extent={{-20,116},{-140,66}},
                  lineColor={0,0,0},
                  textString=DynamicSelect("", String(m_flow, leftjustified=false, significantDigits=3)))}),
          Documentation(info="<html>
<p>
This model outputs the mass flow rate flowing from
<code>port_a</code> to <code>port_b</code>.
The sensor is ideal, i.e., it does not influence the fluid.
</p>
</html>",           revisions="<html>
<ul>
<li>
February 25, 2020, by Michael Wetter:<br/>
Changed icon to display its operating state.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1294\">#1294</a>.
</li>
<li>
February 21, 2020, by Michael Wetter:<br/>
Changed icon to display its operating state.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1294\">#1294</a>.
</li>
<li>
September 29, 2009, by Michael Wetter:<br/>
First implementation.
Implementation is based on <code>Modelica.Fluid</code>.
</li>
</ul>
</html>"));
          end MassFlowRate;

          package BaseClasses "Package with base classes for Buildings.Fluid.Sensors"
            extends Modelica.Icons.BasesPackage;

            partial model PartialFlowSensor
              "Partial component to model sensors that measure flow properties"
              extends Buildings.Fluid.Interfaces.PartialTwoPort;
              parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
                "Nominal mass flow rate, used for regularization near zero flow"
                annotation(Dialog(group = "Nominal condition"));
              parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*m_flow_nominal
                "For bi-directional flow, temperature is regularized in the region |m_flow| < m_flow_small (m_flow_small > 0 required)"
                annotation(Dialog(tab="Advanced"));
            equation
              // mass balance
              port_b.m_flow = -port_a.m_flow;
              // momentum equation (no pressure loss)
              port_a.p = port_b.p;
              // isenthalpic state transformation (no storage and no loss of energy)
              port_a.h_outflow = if allowFlowReversal then inStream(port_b.h_outflow) else Medium.h_default;
              port_b.h_outflow = inStream(port_a.h_outflow);
              port_a.Xi_outflow = if allowFlowReversal then inStream(port_b.Xi_outflow) else Medium.X_default[1:Medium.nXi];
              port_b.Xi_outflow = inStream(port_a.Xi_outflow);
              port_a.C_outflow = if allowFlowReversal then inStream(port_b.C_outflow) else zeros(Medium.nC);
              port_b.C_outflow = inStream(port_a.C_outflow);
              annotation (Documentation(info="<html>
<p>
Partial component to model a sensor.
The sensor is ideal. It does not influence mass, energy,
species or substance balance, and it has no flow friction.
</p>
</html>",   revisions="<html>
<ul>
<li>
August 15, 2015, by Filip Jorissen:<br/>
Implemented more efficient computation of <code>port_a.Xi_outflow</code>,
<code>port_a.h_outflow</code>
and <code>port_a.C_outflow</code> when <code>allowFlowReversal=false</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/281\">#281</a>.
</li>
<li>
June 19, 2015, by Michael Wetter:<br/>
Moved <code>m_flow_small</code> to the <code>Advanced</code> tab
as it usually need not be changed by the user.
Other models such as heat exchangers also have this parameter
on the <code>Advanced</code> tab.
</li>
<li>
February 12, 2011, by Michael Wetter:<br/>
First implementation.
Implementation is based on <code>Modelica.Fluid</code>.
</li>
</ul>
</html>"));
            end PartialFlowSensor;
          annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Sensors\">Buildings.Fluid.Sensors</a>.
</p>
</html>"));
          end BaseClasses;
        annotation (preferredView="info",
        Documentation(info="<html>
<p>
Package <code>Sensors</code> consists of idealized sensor components that
provide variables of a medium as
output signals. These signals can be, e.g., further processed
with components of the
<a href=\"modelica://Modelica.Blocks\">
Modelica.Blocks</a>
library.
</p>
</html>",     revisions="<html>
<ul>
<li><i>22 Dec 2008</i>
    by R&uuml;diger Franke
    <ul>
    <li>flow sensors based on Modelica.Fluid.Interfaces.PartialTwoPort</li>
    <li>adapted documentation to stream connectors, i.e. less need for two port sensors</li>
    </ul>
</li>
<li><i>4 Dec 2008</i>
    by Michael Wetter<br/>
       included sensors for trace substance</li>
<li><i>31 Oct 2007</i>
    by Carsten Heinrich<br/>
       updated sensor models, included one and two port sensors for thermodynamic state variables</li>
</ul>
</html>"));
        end Sensors;

        package Sources "Package with boundary condition models"
          extends Modelica.Icons.SourcesPackage;

          model Boundary_pT
            "Boundary with prescribed pressure, temperature, composition and trace substances"
            extends Buildings.Fluid.Sources.BaseClasses.PartialSource_Xi_C;

            parameter Boolean use_p_in = false
              "Get the pressure from the input connector"
              annotation(Evaluate=true, HideResult=true, Dialog(group="Conditional inputs"));
            parameter Medium.AbsolutePressure p = Medium.p_default
              "Fixed value of pressure"
              annotation (Dialog(enable = not use_p_in, group="Fixed inputs"));

            parameter Boolean use_T_in= false
              "Get the temperature from the input connector"
              annotation(Evaluate=true, HideResult=true,Dialog(group="Conditional inputs"));
            parameter Medium.Temperature T = Medium.T_default
              "Fixed value of temperature"
              annotation (Dialog(enable = not use_T_in,group="Fixed inputs"));

            Modelica.Blocks.Interfaces.RealInput p_in(final unit="Pa") if use_p_in
              "Prescribed boundary pressure"
              annotation (Placement(transformation(extent={{-140,60},{-100,100}})));

            Modelica.Blocks.Interfaces.RealInput T_in(final unit="K",
                                                      displayUnit="degC") if use_T_in
              "Prescribed boundary temperature"
              annotation (Placement(transformation(extent={{-140,20},{-100,60}})));

              // Boolean constants to avoid a potential string comparison in an equation section
          protected
            constant Boolean checkWaterPressure = Medium.mediumName == "SimpleLiquidWater"
              "Evaluates to true if the pressure should be checked";
            constant Boolean checkAirPressure = Medium.mediumName == "Air"
              "Evaluates to true if the pressure should be checked";

            Modelica.Blocks.Interfaces.RealInput T_in_internal(final unit="K",
                                                               displayUnit="degC")
              "Needed to connect to conditional connector";
            Modelica.Blocks.Interfaces.RealInput h_internal=
              Medium.specificEnthalpy(Medium.setState_pTX(p_in_internal, T_in_internal, X_in_internal))
              "Internal connector for enthalpy";

          initial equation
            if not use_p_in then
              if checkWaterPressure then
                assert(p_in_internal>1e4, "In "+getInstanceName() +
                  ": The parameter value p="+String(p_in_internal)+" is low for water. This is likely an error.");
              end if;
              if checkAirPressure then
                assert(p_in_internal>5e4 and p_in_internal < 1.5e5, "In "+getInstanceName() +
                  ": The parameter value p="+String(p_in_internal)+" is not within a realistic range for air. This is likely an error.");
              end if;
            end if;
          equation
            if use_p_in then
              if checkWaterPressure then
                assert(p_in_internal>1e4, "In "+getInstanceName() +
                  ": The value of p_in="+String(p_in_internal)+" is low for water. This is likely an error.");
              end if;
              if checkAirPressure then
                assert(p_in_internal>5e4 and p_in_internal < 1.5e5, "In "+getInstanceName() +
                  ": The value of p_in="+String(p_in_internal)+" is not within a realistic range for air. This is likely an error.");
              end if;
            end if;
            // Pressure
            connect(p_in, p_in_internal);
            if not use_p_in then
              p_in_internal = p;
            end if;
            for i in 1:nPorts loop
              ports[i].p = p_in_internal;
            end for;

            // Temperature
            connect(T_in, T_in_internal);
            if not use_T_in then
              T_in_internal = T;
            end if;
            for i in 1:nPorts loop
               ports[i].h_outflow  = h_internal;
            end for;
            connect(medium.h, h_internal);
            annotation (defaultComponentName="bou",
              Documentation(info="<html>
<p>
Defines prescribed values for boundary conditions:
</p>
<ul>
<li> Prescribed boundary pressure.</li>
<li> Prescribed boundary temperature.</li>
<li> Boundary composition (only for multi-substance or trace-substance flow).</li>
</ul>
<h4>Typical use and important parameters</h4>
<p>
If <code>use_p_in</code> is false (default option),
the <code>p</code> parameter is used as boundary pressure,
and the <code>p_in</code> input connector is disabled;
if <code>use_p_in</code> is true, then the <code>p</code>
parameter is ignored, and the value provided by the
input connector is used instead.
</p>
<p>
The same applies to the temperature <i>T</i>, composition <i>X<sub>i</sub></i> or <i>X</i> and trace substances <i>C</i>.
</p>
<h4>Options</h4>
<p>
Instead of using <code>Xi_in</code> (the <i>independent</i> composition fractions),
the advanced tab provides an option for setting all
composition fractions using <code>X_in</code>.
<code>use_X_in</code> and <code>use_Xi_in</code> cannot be used
at the same time.
</p>
<p>
Parameter <code>verifyInputs</code> can be set to <code>true</code>
to enable a check that verifies the validity of the used temperatures
and pressures.
This removes the corresponding overhead from the model, which is
a substantial part of the overhead of this model.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/882\">#882</a>
for more information.
</p>
<p>
Note, that boundary temperature,
mass fractions and trace substances have only an effect if the mass flow
is from the boundary into the port. If mass is flowing from
the port into the boundary, the boundary definitions,
with exception of boundary pressure, do not have an effect.
</p>
</html>", revisions="<html>
<ul>
<li>
February 25, 2020, by Michael Wetter:<br/>
Changed icon to display its operating state.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1294\">#1294</a>.
</li>
<li>
Juni 7, 2019, by Michael Wetter:<br/>
Added constant boolean expressions to avoid a potential string comparison in an equation section.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1148\">#1148</a>.
</li>
<li>
Juni 4, 2019, by Filip Jorissen:<br/>
Added check for the value of <code>p</code> and <code>p_in</code>.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1148\">#1148</a>.
</li>
<li>
January 25, 2019, by Michael Wetter:<br/>
Refactored use of base classes.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1072\">#1072</a>.
</li>
<li>
February 2nd, 2018 by Filip Jorissen<br/>
Made <code>medium</code> conditional and refactored inputs.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/882\">#882</a>.
</li>
<li>
April 18, 2017, by Filip Jorissen:<br/>
Changed <code>checkBoundary</code> implementation
such that it is run as an initial equation
when it depends on parameters only.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/728\">#728</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Added <code>unit</code> and <code>quantity</code> attributes.
</li>
<li>
May 29, 2014, by Michael Wetter:<br/>
Removed undesirable annotation <code>Evaluate=true</code>.
</li>
<li>
September 29, 2009, by Michael Wetter:<br/>
First implementation.
Implementation is based on <code>Modelica.Fluid</code>.
</li>
</ul>
</html>"),    Icon(graphics={
                  Text(
                    visible=use_p_in,
                    extent={{-152,134},{-68,94}},
                    lineColor={0,0,0},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    textString="p"),
                  Text(
                    visible=use_T_in,
                    extent={{-162,34},{-60,-6}},
                    lineColor={0,0,0},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    textString="T"),
                  Text(
                    extent={{-150,110},{150,150}},
                    textString="%name",
                    lineColor={0,0,255}),
                  Ellipse(
                    extent={{-100,100},{100,-100}},
                    lineColor={0,0,0},
                    fillPattern=FillPattern.Sphere,
                    fillColor=DynamicSelect({0,127,255},
                      min(1, max(0, (1-((if use_T_in then T_in else T)-273.15)/50)))*{28,108,200}+
                      min(1, max(0, ((if use_T_in then T_in else T)-273.15)/50))*{255,0,0})),
                  Text(
                    extent={{62,28},{-58,-22}},
                    lineColor={255,255,255},
                    textString=DynamicSelect("", String((if use_T_in then T_in else T)-273.15, format=".1f")))}));
          end Boundary_pT;

          package BaseClasses "Package with base classes for Buildings.Fluid.Sources"
            extends Modelica.Icons.BasesPackage;

            partial model PartialSource
              "Partial component source with one fluid connector"

              replaceable package Medium =
                Modelica.Media.Interfaces.PartialMedium "Medium in the component"
                  annotation (choices(
                    choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
                    choice(redeclare package Medium = Buildings.Media.Water "Water"),
                    choice(redeclare package Medium =
                        Buildings.Media.Antifreeze.PropyleneGlycolWater (
                          property_T=293.15,
                          X_a=0.40)
                          "Propylene glycol water, 40% mass fraction")));

              parameter Integer nPorts=0 "Number of ports" annotation(Dialog(connectorSizing=true));
              parameter Boolean verifyInputs = false
                "Set to true to stop the simulation with an error if the medium temperature is outside its allowable range"
                annotation(Evaluate=true, Dialog(tab="Advanced"));

              Modelica.Fluid.Interfaces.FluidPorts_b ports[nPorts](
                redeclare each package Medium = Medium,
                m_flow(each max=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Leaving
                         then 0 else +Modelica.Constants.inf,
                       each min=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Entering
                       then 0 else -Modelica.Constants.inf))
                annotation (Placement(transformation(extent={{90,40},{110,-40}})));

            protected
              parameter Modelica.Fluid.Types.PortFlowDirection flowDirection=Modelica.Fluid.Types.PortFlowDirection.Bidirectional
                "Allowed flow direction" annotation (Evaluate=true, Dialog(tab="Advanced"));
              Modelica.Blocks.Interfaces.RealInput p_in_internal(final unit="Pa")
                "Needed to connect to conditional connector";
              Medium.BaseProperties medium if verifyInputs "Medium in the source";
              Modelica.Blocks.Interfaces.RealInput Xi_in_internal[Medium.nXi](
                each final unit = "kg/kg")
                "Needed to connect to conditional connector";
              Modelica.Blocks.Interfaces.RealInput X_in_internal[Medium.nX](
                each final unit = "kg/kg")
                "Needed to connect to conditional connector";
              Modelica.Blocks.Interfaces.RealInput C_in_internal[Medium.nC](
                final quantity=Medium.extraPropertiesNames)
                "Needed to connect to conditional connector";

            initial equation
              // Only one connection allowed to a port to avoid unwanted ideal mixing
              for i in 1:nPorts loop
                assert(cardinality(ports[i]) <= 1,"
Each ports[i] of boundary shall at most be connected to one component.
If two or more connections are present, ideal mixing takes
place in these connections, which is usually not the intention
of the modeller. Increase nPorts to add an additional port.
");           end for;

            equation
              connect(medium.p, p_in_internal);

              annotation (defaultComponentName="bou",
              Documentation(info="<html>
<p>
Partial model for a fluid source that either prescribes
pressure or mass flow rate.
Models that extend this partial model need to prescribe the outflowing
specific enthalpy, composition and trace substances.
This partial model only declares the <code>ports</code>
and ensures that the pressures at all ports are equal.
</p>
<h4>Implementation</h4>
<p>
If the parameter <code>verifyInputs</code> is set to <code>true</code>,
then a protected instance of medium base properties is enabled.
This instance verifies that the
medium temperature is within the bounds <code>T_min</code> and <code>T_max</code>,
where <code>T_min</code> and <code>T_max</code> are constants of the <code>Medium</code>.
If the temperature is outside these bounds, the simulation will stop with an error.
</p>
</html>",             revisions="<html>
<ul>
<li>
January 18, 2019, by Jianjun Hu:<br/>
Limited the media choice.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1050\">#1050</a>.
</li>
<li>
May 30, 2018, by Michael Wetter:<br/>
Improved documentation.
</li>
<li>
February 2nd, 2018 by Filip Jorissen<br/>
Initial version for refactoring inputs of sources.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/882\">#882</a>.
</li>
</ul>
</html>"));
            end PartialSource;

            partial model PartialSource_Xi_C
              "Partial component source with parameter definitions for Xi and C"
              extends Buildings.Fluid.Sources.BaseClasses.PartialSource;

              parameter Boolean use_X_in = false
                "Get the composition (all fractions) from the input connector"
                annotation(Evaluate=true, HideResult=true, Dialog(tab="Advanced"));
              parameter Boolean use_Xi_in = false
                "Get the composition (independent fractions) from the input connector"
                annotation(Evaluate=true, HideResult=true, Dialog(group="Conditional inputs"));
              parameter Boolean use_C_in = false
                "Get the trace substances from the input connector"
                annotation(Evaluate=true, HideResult=true, Dialog(group="Conditional inputs"));
              parameter Medium.MassFraction X[Medium.nX](
                final quantity=Medium.substanceNames) = Medium.X_default
                "Fixed value of composition"
                annotation (Dialog(enable = (not use_X_in) and Medium.nXi > 0, group="Fixed inputs"));
              parameter Medium.ExtraProperty C[Medium.nC](
                final quantity=Medium.extraPropertiesNames) = fill(0, Medium.nC)
                "Fixed values of trace substances"
                annotation (Dialog(enable = (not use_C_in) and Medium.nC > 0, group="Fixed inputs"));
              Modelica.Blocks.Interfaces.RealInput X_in[Medium.nX](
                each final unit = "kg/kg",
                final quantity=Medium.substanceNames) if use_X_in
                "Prescribed boundary composition"
                annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
              Modelica.Blocks.Interfaces.RealInput Xi_in[Medium.nXi](
                each final unit = "kg/kg",
                final quantity=Medium.substanceNames[1:Medium.nXi]) if use_Xi_in
                "Prescribed boundary composition"
                annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
              Modelica.Blocks.Interfaces.RealInput C_in[Medium.nC](
                final quantity=Medium.extraPropertiesNames) if use_C_in
                "Prescribed boundary trace substances"
                annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));

            initial equation
              assert(not use_X_in or not use_Xi_in,
                "Cannot use both X and Xi inputs, choose either use_X_in or use_Xi_in.");

              if not use_X_in and not use_Xi_in then
                Modelica.Fluid.Utilities.checkBoundary(Medium.mediumName, Medium.substanceNames,
                  Medium.singleState, true, X_in_internal, "Boundary_pT");
              end if;

            equation
              if use_X_in or use_Xi_in then
                Modelica.Fluid.Utilities.checkBoundary(Medium.mediumName, Medium.substanceNames,
                  Medium.singleState, true, X_in_internal, "Boundary_pT");
              end if;

              // Assign Xi_in_internal and X_in_internal
              // Note that at most one of X_in or Xi_in is present
              connect(X_in, X_in_internal);
              connect(Xi_in, Xi_in_internal);

              if use_Xi_in then
                // Must assign all components of X_in_internal, using Xi_in
                X_in_internal[1:Medium.nXi] = Xi_in_internal[1:Medium.nXi];
                // If reducedX = true, medium contains the equation sum(X) = 1.0
                // Media with only one substance (e.g., water) have reducedX=true
                // FlueGas and SimpleNaturalGas has reducedX = false
                if Medium.reducedX then
                  X_in_internal[Medium.nX] = 1-sum(Xi_in_internal);
                end if;
              elseif use_X_in then
                X_in_internal[1:Medium.nXi] = Xi_in_internal[1:Medium.nXi];
              else
                // No connector is used. Use parameter X.
                X_in_internal = X;
                Xi_in_internal = X[1:Medium.nXi];
              end if;

              connect(C_in, C_in_internal);

              if not use_C_in then
                C_in_internal = C;
              end if;

              for i in 1:nPorts loop
                ports[i].Xi_outflow = Xi_in_internal;
                ports[i].C_outflow = C_in_internal;
              end for;

              annotation (
                Icon(coordinateSystem(
                    preserveAspectRatio=true,
                    extent={{-100,-100},{100,100}},
                    grid={1,1}), graphics={
                    Text(
                      visible=use_X_in,
                      extent={{-164,4},{-62,-36}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      textString="X"),
                    Text(
                      visible=use_Xi_in,
                      extent={{-164,4},{-62,-36}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      textString="Xi"),
                    Text(
                      visible=use_C_in,
                      extent={{-164,-90},{-62,-130}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      textString="C")}),
                      Documentation(info="<html>
<p>
Partial model that defines outflowing properties
<code>ports.Xi_outflow</code> and <code>ports.C_outflow</code>
using an optional input for both.
Otherwise the parameter value is used.
</p>
</html>",             revisions="<html>
<ul>
<li>
September 19, 2019, by Michael Wetter:<br/>
Refactored handling of mass fractions which was needed to handle media such as
<a href=\"modelica://Modelica.Media.IdealGases.MixtureGases.FlueGasSixComponents\">
Modelica.Media.IdealGases.MixtureGases.FlueGasSixComponents</a> and
<a href=\"modelica://Modelica.Media.IdealGases.MixtureGases.SimpleNaturalGas\">
Modelica.Media.IdealGases.MixtureGases.SimpleNaturalGas</a>.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1205\">Buildings, #1205</a>.
</li>
<li>
February 13, 2018, by Michael Wetter:<br/>
Corrected error in quantity assignment for <code>Xi_in</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/882\">#882</a>.
</li>
<li>
February 2nd, 2018 by Filip Jorissen<br/>
Initial version for refactoring inputs of sources.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/882\">#882</a>.
</li>
</ul>
</html>"));
            end PartialSource_Xi_C;
          annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Sources\">Buildings.Fluid.Sources</a>.
</p>
</html>"));
          end BaseClasses;
        annotation (preferredView="info",
        Documentation(info="<html>
<p>
Package <b>Sources</b> contains generic sources for fluid connectors
to define fixed or prescribed ambient conditions.
</p>
</html>"));
        end Sources;

        package Interfaces "Package with interfaces for fluid models"
          extends Modelica.Icons.InterfacesPackage;

          model ConservationEquation "Lumped volume with mass and energy balance"

            extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;

            // Constants
            parameter Boolean initialize_p = not Medium.singleState
              "= true to set up initial equations for pressure"
              annotation(HideResult=true, Evaluate=true, Dialog(tab="Advanced"));

            constant Boolean simplify_mWat_flow = true
              "Set to true to cause port_a.m_flow + port_b.m_flow = 0 even if mWat_flow is non-zero. Used only if Medium.nX > 1";

            // Port definitions
            parameter Integer nPorts=0 "Number of ports"
              annotation(Evaluate=true, Dialog(connectorSizing=true, tab="General",group="Ports"));

            parameter Boolean use_mWat_flow = false
              "Set to true to enable input connector for moisture mass flow rate"
              annotation(Evaluate=true, Dialog(tab="Advanced"));
            parameter Boolean use_C_flow = false
              "Set to true to enable input connector for trace substance"
              annotation(Evaluate=true, Dialog(tab="Advanced"));

            Modelica.Blocks.Interfaces.RealInput Q_flow(unit="W")
              "Sensible plus latent heat flow rate transferred into the medium"
              annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
            Modelica.Blocks.Interfaces.RealInput mWat_flow(final quantity="MassFlowRate",
                                                           unit="kg/s") if
                 use_mWat_flow "Moisture mass flow rate added to the medium"
              annotation (Placement(transformation(extent={{-140,0},{-100,40}})));
            Modelica.Blocks.Interfaces.RealInput[Medium.nC] C_flow if
                 use_C_flow "Trace substance mass flow rate added to the medium"
              annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));

            // Outputs that are needed in models that use this model
            Modelica.Blocks.Interfaces.RealOutput hOut(unit="J/kg",
                                                       start=hStart)
              "Leaving specific enthalpy of the component"
               annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={-50,110})));
            Modelica.Blocks.Interfaces.RealOutput XiOut[Medium.nXi](each unit="1",
                                                                    each min=0,
                                                                    each max=1)
              "Leaving species concentration of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={0,110})));
            Modelica.Blocks.Interfaces.RealOutput COut[Medium.nC](each min=0)
              "Leaving trace substances of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={50,110})));
            Modelica.Blocks.Interfaces.RealOutput UOut(unit="J")
              "Internal energy of the component" annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  origin={110,20})));
            Modelica.Blocks.Interfaces.RealOutput mXiOut[Medium.nXi](each min=0, each unit=
                 "kg") "Species mass of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  origin={110,-20})));
            Modelica.Blocks.Interfaces.RealOutput mOut(min=0, unit="kg")
              "Mass of the component" annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  origin={110,60})));
            Modelica.Blocks.Interfaces.RealOutput mCOut[Medium.nC](each min=0, each unit="kg")
              "Trace substance mass of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  origin={110,-60})));

            Modelica.Fluid.Vessels.BaseClasses.VesselFluidPorts_b ports[nPorts](
                redeclare each final package Medium = Medium) "Fluid inlets and outlets"
              annotation (Placement(transformation(extent={{-40,-10},{40,10}},
                origin={0,-100})));

            // Set nominal attributes where literal values can be used.
            Medium.BaseProperties medium(
              p(start=p_start),
              h(start=hStart),
              T(start=T_start),
              Xi(start=X_start[1:Medium.nXi]),
              X(start=X_start),
              d(start=rho_start)) "Medium properties";

            Modelica.SIunits.Energy U(start=fluidVolume*rho_start*
              Medium.specificInternalEnergy(Medium.setState_pTX(
               T=T_start,
               p=p_start,
               X=X_start[1:Medium.nXi])) +
              (T_start - Medium.reference_T)*CSen,
              nominal = 1E5) "Internal energy of fluid";

            Modelica.SIunits.Mass m(
              start=fluidVolume*rho_start,
              stateSelect=if massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState
              then StateSelect.default else StateSelect.prefer)
              "Mass of fluid";

            Modelica.SIunits.Mass[Medium.nXi] mXi(
              start=fluidVolume*rho_start*X_start[1:Medium.nXi])
              "Masses of independent components in the fluid";
            Modelica.SIunits.Mass[Medium.nC] mC(
              start=fluidVolume*rho_start*C_start)
              "Masses of trace substances in the fluid";
            // C need to be added here because unlike for Xi, which has medium.Xi,
            // there is no variable medium.C
            Medium.ExtraProperty C[Medium.nC](nominal=C_nominal)
              "Trace substance mixture content";

            Modelica.SIunits.MassFlowRate mb_flow "Mass flows across boundaries";
            Modelica.SIunits.MassFlowRate[Medium.nXi] mbXi_flow
              "Substance mass flows across boundaries";
            Medium.ExtraPropertyFlowRate[Medium.nC] mbC_flow
              "Trace substance mass flows across boundaries";
            Modelica.SIunits.EnthalpyFlowRate Hb_flow
              "Enthalpy flow across boundaries or energy source/sink";

            // Parameters that need to be defined by an extending class
            parameter Modelica.SIunits.Volume fluidVolume "Volume";
            final parameter Modelica.SIunits.HeatCapacity CSen=
              (mSenFac - 1)*rho_default*cp_default*fluidVolume
              "Aditional heat capacity for implementing mFactor";
          protected
            Medium.EnthalpyFlowRate ports_H_flow[nPorts];
            Modelica.SIunits.MassFlowRate ports_mXi_flow[nPorts,Medium.nXi];
            Medium.ExtraPropertyFlowRate ports_mC_flow[nPorts,Medium.nC];
            parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
            Medium.specificHeatCapacityCp(state=state_default)
              "Heat capacity, to compute additional dry mass";
            parameter Modelica.SIunits.Density rho_start=Medium.density(
             Medium.setState_pTX(
               T=T_start,
               p=p_start,
               X=X_start[1:Medium.nXi])) "Density, used to compute fluid mass";

            // Parameter for avoiding extra overhead calculations when CSen==0
            final parameter Boolean computeCSen = abs(mSenFac-1) > Modelica.Constants.eps
              annotation(Evaluate=true);
            final parameter Medium.ThermodynamicState state_default = Medium.setState_pTX(
                T=Medium.T_default,
                p=Medium.p_default,
                X=Medium.X_default[1:Medium.nXi]) "Medium state at default values";
            // Density at medium default values, used to compute the size of control volumes
            final parameter Modelica.SIunits.Density rho_default=Medium.density(
              state=state_default) "Density, used to compute fluid mass";
            // Parameter that is used to construct the vector mXi_flow
            final parameter Real s[Medium.nXi] = {if Modelica.Utilities.Strings.isEqual(
                                                      string1=Medium.substanceNames[i],
                                                      string2="Water",
                                                      caseSensitive=false)
                                                      then 1 else 0 for i in 1:Medium.nXi}
              "Vector with zero everywhere except where species is";
            parameter Modelica.SIunits.SpecificEnthalpy hStart=
              Medium.specificEnthalpy_pTX(p_start, T_start, X_start)
              "Start value for specific enthalpy";

            // Set _simplify_mWat_flow == false for Glycol47; otherwise Dymola 2018FD01
            // cannot differentiate the equations.
            constant Boolean _simplify_mWat_flow = simplify_mWat_flow and Medium.nX > 1
             "If true, then port_a.m_flow + port_b.m_flow = 0 even if mWat_flow is non-zero, and equations are simplified";

            // Conditional connectors
            Modelica.Blocks.Interfaces.RealInput mWat_flow_internal(unit="kg/s")
              "Needed to connect to conditional connector";
            Modelica.Blocks.Interfaces.RealInput C_flow_internal[Medium.nC]
              "Needed to connect to conditional connector";

          initial equation
            // Assert that the substance with name 'water' has been found.
            assert(Medium.nXi == 0 or abs(sum(s)-1) < 1e-5,
                "In " + getInstanceName() + ":
         If Medium.nXi > 1, then substance 'water' must be present for one component of '"
                   + Medium.mediumName + "'.
         Check medium model.");

            // Make sure that if energyDynamics is SteadyState, then
            // massDynamics is also SteadyState.
            // Otherwise, the system of ordinary differential equations may be inconsistent.
            if energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then
              assert(massDynamics == energyDynamics, "In " + getInstanceName() + ":
         If 'massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState', then it is
         required that 'energyDynamics==Modelica.Fluid.Types.Dynamics.SteadyState'.
         Otherwise, the system of equations may not be consistent.
         You need to select other parameter values.");
            end if;

            // initialization of balances
            if energyDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial then
                medium.T = T_start;
            else
              if energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyStateInitial then
                  der(medium.T) = 0;
              end if;
            end if;

            if massDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial then
              if initialize_p then
                medium.p = p_start;
              end if;
            else
              if massDynamics == Modelica.Fluid.Types.Dynamics.SteadyStateInitial then
                if initialize_p then
                  der(medium.p) = 0;
                end if;
              end if;
            end if;

            if substanceDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial then
              medium.Xi = X_start[1:Medium.nXi];
            else
              if substanceDynamics == Modelica.Fluid.Types.Dynamics.SteadyStateInitial then
                der(medium.Xi) = zeros(Medium.nXi);
              end if;
            end if;

            if traceDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial then
              C = C_start[1:Medium.nC];
            else
              if traceDynamics == Modelica.Fluid.Types.Dynamics.SteadyStateInitial then
                der(C) = zeros(Medium.nC);
              end if;
            end if;

          equation
            // Conditional connectors
            connect(mWat_flow, mWat_flow_internal);
            if not use_mWat_flow then
              mWat_flow_internal = 0;
            end if;

            connect(C_flow, C_flow_internal);
            if not use_C_flow then
              C_flow_internal = zeros(Medium.nC);
            end if;

            // Total quantities
            if massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then
              m = fluidVolume*rho_start;
            else
              if _simplify_mWat_flow then
                // If moisture is neglected in mass balance, assume for computation
                // of the mass of air that the air is at Medium.X_default.
                m = fluidVolume*Medium.density(Medium.setState_phX(
                  p = medium.p,
                  h = hOut,
                  X = Medium.X_default));
              else
                // Use actual density
                m = fluidVolume*medium.d;
              end if;
            end if;
            mXi = m*medium.Xi;
            if computeCSen then
              U = m*medium.u + CSen*(medium.T-Medium.reference_T);
            else
              U = m*medium.u;
            end if;
            mC = m*C;

            hOut = medium.h;
            XiOut = medium.Xi;
            COut = C;

            for i in 1:nPorts loop
              //The semiLinear function should be used for the equations below
              //for allowing min/max simplifications.
              //See https://github.com/ibpsa/modelica-ibpsa/issues/216 for a discussion and motivation
              ports_H_flow[i]     = semiLinear(ports[i].m_flow, inStream(ports[i].h_outflow), ports[i].h_outflow)
                "Enthalpy flow";
              for j in 1:Medium.nXi loop
                ports_mXi_flow[i,j] = semiLinear(ports[i].m_flow, inStream(ports[i].Xi_outflow[j]), ports[i].Xi_outflow[j])
                  "Component mass flow";
              end for;
              for j in 1:Medium.nC loop
                ports_mC_flow[i,j]  = semiLinear(ports[i].m_flow, inStream(ports[i].C_outflow[j]),  ports[i].C_outflow[j])
                  "Trace substance mass flow";
              end for;
            end for;

            for i in 1:Medium.nXi loop
              mbXi_flow[i] = sum(ports_mXi_flow[:,i]);
            end for;

            for i in 1:Medium.nC loop
              mbC_flow[i]  = sum(ports_mC_flow[:,i]);
            end for;

            mb_flow = sum(ports.m_flow);
            Hb_flow = sum(ports_H_flow);

            // Energy and mass balances
            if energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then
              0 = Hb_flow + Q_flow;
            else
              der(U) = Hb_flow + Q_flow;
            end if;

            if massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then
              0 = mb_flow + (if simplify_mWat_flow then 0 else mWat_flow_internal);
            else
              der(m) = mb_flow + (if simplify_mWat_flow then 0 else mWat_flow_internal);
            end if;

            if substanceDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then
              zeros(Medium.nXi) = mbXi_flow + mWat_flow_internal * s;
            else
              der(mXi) = mbXi_flow + mWat_flow_internal * s;
            end if;

            if traceDynamics == Modelica.Fluid.Types.Dynamics.SteadyState then
              zeros(Medium.nC)  = mbC_flow + C_flow_internal;
            else
              der(mC)  = mbC_flow + C_flow_internal;
            end if;

            // Properties of outgoing flows
            for i in 1:nPorts loop
                ports[i].p          = medium.p;
                ports[i].h_outflow  = medium.h;
                ports[i].Xi_outflow = medium.Xi;
                ports[i].C_outflow  = C;
            end for;
            UOut=U;
            mXiOut=mXi;
            mOut=m;
            mCOut=mC;
            annotation (
              Documentation(info="<html>
<p>
Basic model for an ideally mixed fluid volume with the ability to store mass and energy.
It implements a dynamic or a steady-state conservation equation for energy and mass fractions.
The model has zero pressure drop between its ports.
</p>
<p>
If the constant <code>simplify_mWat_flow = true</code> then adding
moisture does not increase the mass of the volume or the leaving mass flow rate.
It does however change the mass fraction <code>medium.Xi</code>.
This allows to decouple the moisture balance from the pressure drop equations.
If <code>simplify_mWat_flow = false</code>, then
the outlet mass flow rate is
<i>m<sub>out</sub> = m<sub>in</sub>  (1 + &Delta; X<sub>w</sub>)</i>,
where
<i>&Delta; X<sub>w</sub></i> is the change in water vapor mass
fraction across the component. In this case,
this component couples
the energy calculation to the
pressure drop versus mass flow rate calculations.
However, in typical building HVAC systems,
<i>&Delta; X<sub>w</sub></i> &lt; <i>0.005</i> kg/kg.
Hence, by tolerating a relative error of <i>0.005</i> in the mass balance,
one can decouple these equations.
Decoupling these equations avoids having
to compute the energy balance of the humidifier
and its upstream components when solving for the
pressure drop of downstream components.
Therefore, the default value is <code>simplify_mWat_flow = true</code>.
</p>
<h4>Typical use and important parameters</h4>
<p>
Set the parameter <code>use_mWat_flow_in=true</code> to enable an
input connector for <code>mWat_flow</code>.
Otherwise, the model uses <code>mWat_flow = 0</code>.
</p>
<p>
If the constant <code>simplify_mWat_flow = true</code>, which is its default value,
then the equation
</p>
<pre>
  port_a.m_flow + port_b.m_flow = - mWat_flow;
</pre>
<p>
is simplified as
</p>
<pre>
  port_a.m_flow + port_b.m_flow = 0;
</pre>
<p>
This causes an error in the mass balance of about <i>0.5%</i>, but generally leads to
simpler equations because the pressure drop equations are then decoupled from the
mass exchange in this component.
The model
<a href=\"modelica://Buildings.Fluid.MixingVolumes.Validation.MixingVolumeAdiabaticCooling\">
Buildings.Fluid.MixingVolumes.Validation.MixingVolumeAdiabaticCooling</a>
shows that the relative error on the temperature difference between these
two options of <code>simplify_mWat_flow</code> is less than
<i>0.1%</i>.
</p>

<h4>Implementation</h4>
<p>
When extending or instantiating this model, the input
<code>fluidVolume</code>, which is the actual volume occupied by the fluid,
needs to be assigned.
For most components, this can be set to a parameter.
</p>
Input connectors of the model are
<ul>
<li>
<code>Q_flow</code>, which is the sensible plus latent heat flow rate added to the medium,
</li>
<li>
<code>mWat_flow</code>, which is the moisture mass flow rate added to the medium, and
</li>
<li>
<code>C_flow</code>, which is the trace substance mass flow rate added to the medium.
</li>
</ul>

<p>
The model can be used as a dynamic model or as a steady-state model.
However, for a steady-state model with exactly two fluid ports connected,
the model
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation\">
Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation</a>
provides a more efficient implementation.
</p>
<p>
For a model that instantiates this model, see
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolume\">
Buildings.Fluid.MixingVolumes.MixingVolume</a>.
</p>
</html>",           revisions="<html>
<ul>
<li>
April 26, 2019, by Filip Jorissen:<br/>
Returning <code>getInstanceName()</code> in asserts.
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1133\">1133</a>.
</li>
<li>
April 16, 2019, by Michael Wetter:<br/>
Changed computation of <code>computeCSen</code> to avoid the volume to become
a structural parameter.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1122\">Buildings, issue 1122</a>.
</li>
<li>
April 16, 2018, by Michael Wetter:<br/>
Reformulated mass calculation so that Dymola can differentiate the equations.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/910\">Buildings, issue 910</a>.
</li>
<li>
November 3, 2017, by Michael Wetter:<br/>
Set <code>start</code> attributes.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/727\">727</a>.
</li>
<li>
October 19, 2017, by Michael Wetter:<br/>
Changed initialization of pressure from a <code>constant</code> to a <code>parameter</code>.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1013\">Buildings, issue 1013</a>.
</li>
<li>
January 27, 2017, by Michael Wetter:<br/>
Added <code>stateSelect</code> for mass <code>m</code>.<br/>
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/642\">
Buildings, #642</a>.
</li>
<li>
December 22, 2016, by Michael Wetter:<br/>
Set nominal value for <code>U</code>.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/637\">637</a>.
</li>
<li>
February 19, 2016 by Filip Jorissen:<br/>
Added outputs UOut, mOut, mXiOut, mCOut for being able to
check conservation of quantities.
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/247\">
issue 247</a>.
</li>
<li>
January 17, 2016, by Michael Wetter:<br/>
Added parameter <code>use_C_flow</code> and converted <code>C_flow</code>
to a conditionally removed connector.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/372\">#372</a>.
</li>
<li>
December 16, 2015, by Michael Wetter:<br/>
Added <code>C_flow</code> to the steady-state trace substance balance,
and removed the units of <code>C_flow</code> to allow for PPM.
</li>
<li>
December 2, 2015, by Filip Jorissen:<br/>
Added input <code>C_flow</code> and code for handling trace substance insertions.
</li>
<li>
September 3, 2015, by Filip Jorissen and Michael Wetter:<br/>
Revised implementation for allowing moisture mass flow rate
to be approximated using parameter <code>simplify_mWat_flow</code>.
This may lead to smaller algebraic loops.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/247\">#247</a>.
</li>
<li>
July 17, 2015, by Michael Wetter:<br/>
Added constant <code>simplify_mWat_flow</code> to remove dependencies of the pressure drop
calculation on the moisture balance.
</li>
<li>
June 5, 2015 by Michael Wetter:<br/>
Removed <code>preferredMediumStates= false</code> in
the instance <code>medium</code> as the default
is already <code>false</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/260\">#260</a>.
</li>
<li>
June 5, 2015 by Filip Jorissen:<br/>
Removed <pre>
Xi(start=X_start[1:Medium.nXi],
       each stateSelect=if (not (substanceDynamics == Modelica.Fluid.Types.Dynamics.SteadyState))
       then StateSelect.prefer else StateSelect.default),
</pre>
and set
<code>preferredMediumStates = false</code>
because the previous declaration led to more equations and
translation problems in large models.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/260\">#260</a>.
</li>
<li>
June 5, 2015, by Michael Wetter:<br/>
Moved assignment of <code>dynBal.U.start</code>
from instance <code>dynBal</code> of <code>PartialMixingVolume</code>
to this model implementation.
This is required for a pedantic model check in Dymola 2016.
It addresses
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/266\">
issue 266</a>.
This revison also renames the protected variable
<code>rho_nominal</code> to <code>rho_start</code>
as it depends on the start values and not the nominal values.
</li>
<li>
May 22, 2015 by Michael Wetter:<br/>
Removed <pre>
p(stateSelect=if not (massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState)
then StateSelect.prefer else StateSelect.default)
</pre>
because the previous declaration led to the translation error
<pre>
The model requires derivatives of some inputs as listed below:
1 inlet.m_flow
1 inlet.p
</pre>
when translating
<code>Buildings.Fluid.FMI.ExportContainers.Examples.FMUs.HeaterCooler_u</code>
with a dynamic energy balance.
</li>
<li>
May 6, 2015, by Michael Wetter:<br/>
Corrected documentation.
</li>
<li>
April 13, 2015, by Filip Jorissen:<br/>
Now using <code>semiLinear()</code> function for calculation of
<code>ports_H_flow</code>. This enables Dymola to simplify based on
the <code>min</code> and <code>max</code> attribute of the mass flow rate.
</li>
<li>
February 16, 2015, by Filip Jorissen:<br/>
Fixed SteadyState massDynamics implementation for compressible media.
Mass <code>m</code> is now constant.
</li>
<li>
February 5, 2015, by Michael Wetter:<br/>
Changed <code>initalize_p</code> from a <code>parameter</code> to a
<code>constant</code>. This is only required in finite volume models
of heat exchangers (to avoid consistent but redundant initial conditions)
and hence it should be set as a <code>constant</code>.
</li>
<li>
February 3, 2015, by Michael Wetter:<br/>
Removed <code>stateSelect.prefer</code> for temperature.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/160\">#160</a>.
</li>
<li>
October 21, 2014, by Filip Jorissen:<br/>
Added parameter <code>mFactor</code> to increase the thermal capacity.
</li>
<li>
October 6, 2014, by Michael Wetter:<br/>
Changed medium declaration in ports to be final.
</li>
<li>
October 6, 2014, by Michael Wetter:<br/>
Set start attributes in <code>medium</code> to avoid in OpenModelica the warning
alias set with several free start values.
</li>
<li>
October 3, 2014, by Michael Wetter:<br/>
Changed assignment of nominal value to avoid in OpenModelica the warning
alias set with different nominal values.
</li>
<li>
July 3, 2014, by Michael Wetter:<br/>
Added parameter <code>initialize_p</code>. This is required
to enable the coil models to initialize the pressure in the first
volume, but not in the downstream volumes. Otherwise,
the initial equations will be overdetermined, but consistent.
This change was done to avoid a long information message that appears
when translating models.
</li>
<li>
May 29, 2014, by Michael Wetter:<br/>
Removed undesirable annotation <code>Evaluate=true</code>.
</li>
<li>
February 11, 2014 by Michael Wetter:<br/>
Improved documentation for <code>Q_flow</code> input.
</li>
<li>
September 17, 2013 by Michael Wetter:<br/>
Added start value for <code>hOut</code>.
</li>
<li>
September 10, 2013 by Michael Wetter:<br/>
Removed unrequired parameter <code>i_w</code>.<br/>
Corrected the syntax error
<code>Medium.ExtraProperty C[Medium.nC](each nominal=C_nominal)</code>
to
<code>Medium.ExtraProperty C[Medium.nC](nominal=C_nominal)</code>
because <code>C_nominal</code> is a vector.
This syntax error caused a compilation error in OpenModelica.
</li>
<li>
July 30, 2013 by Michael Wetter:<br/>
Changed connector <code>mXi_flow[Medium.nXi]</code>
to a scalar input connector <code>mWat_flow</code>.
The reason is that <code>mXi_flow</code> does not allow
to compute the other components in <code>mX_flow</code> and
therefore leads to an ambiguous use of the model.
By only requesting <code>mWat_flow</code>, the mass balance
and species balance can be implemented correctly.
</li>
<li>
March 27, 2013 by Michael Wetter:<br/>
Removed wrong unit attribute of <code>COut</code>,
and added min and max attributes for <code>XiOut</code>.
</li>
<li>
July 31, 2011 by Michael Wetter:<br/>
Added test to stop model translation if the setting for
<code>energyBalance</code> and <code>massBalance</code>
can lead to inconsistent equations.
</li>
<li>
July 26, 2011 by Michael Wetter:<br/>
Removed the option to use <code>h_start</code>, as this
is not needed for building simulation.
Also removed the reference to <code>Modelica.Fluid.System</code>.
Moved parameters and medium to
<a href=\"Buildings.Fluid.Interfaces.LumpedVolumeDeclarations\">
Buildings.Fluid.Interfaces.LumpedVolumeDeclarations</a>.
</li>
<li>
July 14, 2011 by Michael Wetter:<br/>
Added start value for medium density.
</li>
<li>
March 29, 2011 by Michael Wetter:<br/>
Changed default value for <code>substanceDynamics</code> and
<code>traceDynamics</code> from <code>energyDynamics</code>
to <code>massDynamics</code>.
</li>
<li>
September 28, 2010 by Michael Wetter:<br/>
Changed array index for nominal value of <code>Xi</code>.
</li>
<li>
September 13, 2010 by Michael Wetter:<br/>
Set nominal attributes for medium based on default medium values.
</li>
<li>
July 30, 2010 by Michael Wetter:<br/>
Added parameter <code>C_nominal</code> which is used as the nominal attribute for <code>C</code>.
Without this value, the ODE solver gives wrong results for concentrations around 1E-7.
</li>
<li>
March 21, 2010 by Michael Wetter:<br/>
Changed pressure start value from <code>system.p_start</code>
to <code>Medium.p_default</code> since HVAC models may have water and
air, which are typically at different pressures.
</li>
<li><i>February 6, 2010</i> by Michael Wetter:<br/>
Added to <code>Medium.BaseProperties</code> the initialization
<code>X(start=X_start[1:Medium.nX])</code>. Previously, the initialization
was only done for <code>Xi</code> but not for <code>X</code>, which caused the
medium to be initialized to <code>reference_X</code>, ignoring the value of <code>X_start</code>.
</li>
<li><i>October 12, 2009</i> by Michael Wetter:<br/>
Implemented first version in <code>Buildings</code> library, based on model from
<code>Modelica.Fluid 1.0</code>.
</li>
</ul>
</html>"),    Icon(graphics={            Rectangle(
                    extent={{-100,100},{100,-100}},
                    fillColor={135,135,135},
                    fillPattern=FillPattern.Solid,
                    pattern=LinePattern.None),
                  Text(
                    extent={{-89,17},{-54,34}},
                    lineColor={0,0,127},
                    textString="mWat_flow"),
                  Text(
                    extent={{-89,52},{-54,69}},
                    lineColor={0,0,127},
                    textString="Q_flow"),
                  Line(points={{-56,-73},{81,-73}}, color={255,255,255}),
                  Line(points={{-42,55},{-42,-84}}, color={255,255,255}),
                  Polygon(
                    points={{-42,67},{-50,45},{-34,45},{-42,67}},
                    lineColor={255,255,255},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid),
                  Polygon(
                    points={{87,-73},{65,-65},{65,-81},{87,-73}},
                    lineColor={255,255,255},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid),
                  Line(
                    points={{-42,-28},{-6,-28},{18,4},{40,12},{66,14}},
                    color={255,255,255},
                    smooth=Smooth.Bezier),
                  Text(
                    extent={{-155,-120},{145,-160}},
                    lineColor={0,0,255},
                    textString="%name")}));
          end ConservationEquation;

          partial model PartialTwoPort "Partial component with two ports"
            replaceable package Medium =
              Modelica.Media.Interfaces.PartialMedium "Medium in the component"
                annotation (choices(
                  choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
                  choice(redeclare package Medium = Buildings.Media.Water "Water"),
                  choice(redeclare package Medium =
                      Buildings.Media.Antifreeze.PropyleneGlycolWater (
                        property_T=293.15,
                        X_a=0.40)
                        "Propylene glycol water, 40% mass fraction")));

            parameter Boolean allowFlowReversal = true
              "= false to simplify equations, assuming, but not enforcing, no flow reversal"
              annotation(Dialog(tab="Assumptions"), Evaluate=true);

            Modelica.Fluid.Interfaces.FluidPort_a port_a(
              redeclare final package Medium = Medium,
               m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
               h_outflow(start = Medium.h_default, nominal = Medium.h_default))
              "Fluid connector a (positive design flow direction is from port_a to port_b)"
              annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
            Modelica.Fluid.Interfaces.FluidPort_b port_b(
              redeclare final package Medium = Medium,
              m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
               h_outflow(start = Medium.h_default, nominal = Medium.h_default))
              "Fluid connector b (positive design flow direction is from port_a to port_b)"
              annotation (Placement(transformation(extent={{110,-10},{90,10}})));

            annotation (
              Documentation(info="<html>
<p>
This partial model defines an interface for components with two ports.
The treatment of the design flow direction and of flow reversal are predefined based on the parameter <code>allowFlowReversal</code>.
The component may transport fluid and may have internal storage for a given fluid <code>Medium</code>.
</p>
<h4>Implementation</h4>
<p>
This model is similar to
<a href=\"modelica://Modelica.Fluid.Interfaces.PartialTwoPort\">
Modelica.Fluid.Interfaces.PartialTwoPort</a>
but it does not use the <code>outer system</code> declaration.
This declaration is omitted as in building energy simulation,
many models use multiple media, an in practice,
users have not used this global definition to assign parameters.
</p>
</html>",           revisions="<html>
<ul>
<li>
January 18, 2019, by Jianjun Hu:<br/>
Limited the media choice.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1050\">#1050</a>.
</li>
<li>
July 8, 2018, by Filip Jorissen:<br/>
Added nominal value of <code>h_outflow</code> in <code>FluidPorts</code>.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/977\">#977</a>.
</li>
<li>
November 19, 2015, by Michael Wetter:<br/>
Removed parameters
<code>port_a_exposesState</code> and
<code>port_b_exposesState</code>
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/351\">#351</a>
and
<code>showDesignFlowDirection</code>
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/349\">#349</a>.
</li>
<li>
November 13, 2015, by Michael Wetter:<br/>
Assinged <code>start</code> attribute for leaving
enthalpy at <code>port_a</code> and <code>port_b</code>.
This was done to make the model similar to
<a href=\"modelica://Buildings.Fluid.Interfaces.PartialFourPort\">
Buildings.Fluid.Interfaces.PartialFourPort</a>.
</li>
<li>
November 12, 2015, by Michael Wetter:<br/>
Removed import statement.
</li>
<li>
October 21, 2014, by Michael Wetter:<br/>
Revised implementation.
Declared medium in ports to be <code>final</code>.
</li>
<li>
October 20, 2014, by Filip Jorisson:<br/>
First implementation.
</li>
</ul>
</html>"),    Icon(coordinateSystem(
                    preserveAspectRatio=true,
                    extent={{-100,-100},{100,100}}), graphics={
                  Polygon(
                    points={{20,-70},{60,-85},{20,-100},{20,-70}},
                    lineColor={0,128,255},
                    fillColor={0,128,255},
                    fillPattern=FillPattern.Solid,
                    visible=not allowFlowReversal),
                  Line(
                    points={{55,-85},{-60,-85}},
                    color={0,128,255},
                    visible=not allowFlowReversal),
                  Text(
                    extent={{-149,-114},{151,-154}},
                    lineColor={0,0,255},
                    textString="%name")}));
          end PartialTwoPort;

          partial model PartialTwoPortInterface
            "Partial model transporting fluid between two ports without storing mass or energy"
            extends Buildings.Fluid.Interfaces.PartialTwoPort(port_a(p(start=Medium.p_default)), port_b(p(start=Medium.p_default)));

            parameter Modelica.SIunits.MassFlowRate m_flow_nominal
              "Nominal mass flow rate"
              annotation(Dialog(group = "Nominal condition"));
            parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
              "Small mass flow rate for regularization of zero flow"
              annotation(Dialog(tab = "Advanced"));
            // Diagnostics
             parameter Boolean show_T = false
              "= true, if actual temperature at port is computed"
              annotation(Dialog(tab="Advanced",group="Diagnostics"));

            Modelica.SIunits.MassFlowRate m_flow(start=_m_flow_start) = port_a.m_flow
              "Mass flow rate from port_a to port_b (m_flow > 0 is design flow direction)";

            Modelica.SIunits.PressureDifference dp(start=_dp_start, displayUnit="Pa") = port_a.p - port_b.p
              "Pressure difference between port_a and port_b";

            Medium.ThermodynamicState sta_a=
                Medium.setState_phX(port_a.p,
                                    noEvent(actualStream(port_a.h_outflow)),
                                    noEvent(actualStream(port_a.Xi_outflow))) if
                   show_T "Medium properties in port_a";

            Medium.ThermodynamicState sta_b=
                Medium.setState_phX(port_b.p,
                                    noEvent(actualStream(port_b.h_outflow)),
                                    noEvent(actualStream(port_b.Xi_outflow))) if
                    show_T "Medium properties in port_b";

          protected
            final parameter Modelica.SIunits.MassFlowRate _m_flow_start = 0
            "Start value for m_flow, used to avoid a warning if not set in m_flow, and to avoid m_flow.start in parameter window";
            final parameter Modelica.SIunits.PressureDifference _dp_start(displayUnit="Pa") = 0
            "Start value for dp, used to avoid a warning if not set in dp, and to avoid dp.start in parameter window";

            annotation (
              preferredView="info",
              Documentation(info="<html>
<p>
This component defines the interface for models that
transports a fluid between two ports. It is similar to
<a href=\"Modelica://Modelica.Fluid.Interfaces.PartialTwoPortTransport\">
Modelica.Fluid.Interfaces.PartialTwoPortTransport</a>, but it does not
include the species balance
</p>
<pre>
  port_b.Xi_outflow = inStream(port_a.Xi_outflow);
</pre>
<p>
Thus, it can be used as a base class for a heat <i>and</i> mass transfer component
</p>
<p>
The model is used by other models in this package that add heat transfer,
mass transfer and pressure drop equations. See for example
<a href=\"modelica://Buildings.Fluid.Interfaces.StaticTwoPortHeatMassExchanger\">
Buildings.Fluid.Interfaces.StaticTwoPortHeatMassExchanger</a>.
</p>
</html>",           revisions="<html>
<ul>
<li>
November 3, 2016, by Michael Wetter:<br/>
Renamed protected parameter <code>m_flow_start</code> to avoid
a name clash with
<a href=\"modelica://Buildings.Fluid.Movers.FlowControlled_m_flow\">
Buildings.Fluid.Movers.FlowControlled_m_flow</a>
which leads to an error as the definition were different,
and also renamed protected parameter <code>dp_start</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/552\">#552</a>
<br/>
Moved computation of pressure drop to variable assignment so that
the model won't mix graphical with textual modeling if used as a base
class for a graphically implemented model.
</li>
<li>
November 3, 2016, by Michael Wetter:<br/>
Removed start values for mass flow rate and pressure difference
to simplify the parameter window.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/552\">#552</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
October 3, 2014, by Michael Wetter:<br/>
Changed assignment of nominal value to avoid in OpenModelica the warning
alias set with different nominal values.
</li>
<li>
November 12, 2013 by Michael Wetter:<br/>
Removed <code>import Modelica.Constants;</code> statement.
</li>
<li>
November 11, 2013 by Michael Wetter:<br/>
Removed the parameter <code>homotopyInitialization</code>
as it is no longer used in this model.
</li>
<li>
November 10, 2013 by Michael Wetter:<br/>
In the computation of <code>sta_a</code> and <code>sta_b</code>,
removed the branch that uses the homotopy operator.
The rational is that these variables are conditionally enables (because
of <code>... if show_T</code>). Therefore, the Modelica Language Specification
does not allow for these variables to be used in any equation. Hence,
the use of the homotopy operator is not needed here.
</li>
<li>
October 10, 2013 by Michael Wetter:<br/>
Added <code>noEvent</code> to the computation of the states at the port.
This is correct, because the states are only used for reporting, but not
to compute any other variable.
Use of the states to compute other variables would violate the Modelica
language, as conditionally removed variables must not be used in any equation.
</li>
<li>
October 8, 2013 by Michael Wetter:<br/>
Removed the computation of <code>V_flow</code> and removed the parameter
<code>show_V_flow</code>.
The reason is that the computation of <code>V_flow</code> required
the use of <code>sta_a</code> (to compute the density),
but <code>sta_a</code> is also a variable that is conditionally
enabled. However, this was not correct Modelica syntax as conditional variables
can only be used in a <code>connect</code>
statement, not in an assignment. Dymola 2014 FD01 beta3 is checking
for this incorrect syntax. Hence, <code>V_flow</code> was removed as its
conditional implementation would require a rather cumbersome implementation
that uses a new connector that carries the state of the medium.
</li>
<li>
April 26, 2013 by Marco Bonvini:<br/>
Moved the definition of <code>dp</code> because it causes some problem with PyFMI.
</li>
<li>
March 27, 2012 by Michael Wetter:<br/>
Changed condition to remove <code>sta_a</code> to also
compute the state at the inlet port if <code>show_V_flow=true</code>.
The previous implementation resulted in a translation error
if <code>show_V_flow=true</code>, but worked correctly otherwise
because the erroneous function call is removed if  <code>show_V_flow=false</code>.
</li>
<li>
March 27, 2011 by Michael Wetter:<br/>
Added <code>homotopy</code> operator.
</li>
<li>
March 21, 2010 by Michael Wetter:<br/>
Changed pressure start value from <code>system.p_start</code>
to <code>Medium.p_default</code> since HVAC models may have water and
air, which are typically at different pressures.
</li>
<li>
September 19, 2008 by Michael Wetter:<br/>
Added equations for the mass balance of extra species flow,
i.e., <code>C</code> and <code>mC_flow</code>.
</li>
<li>
March 11, 2008, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
          end PartialTwoPortInterface;

          partial model PartialTwoPortTransport
            "Partial element transporting fluid between two ports without storage of mass or energy"
            extends Buildings.Fluid.Interfaces.PartialTwoPort;

            // Advanced
            // Note: value of dp_start shall be refined by derived model,
            // based on local dp_nominal
            parameter Modelica.SIunits.PressureDifference dp_start(displayUnit="Pa") = 0
              "Guess value of dp = port_a.p - port_b.p"
              annotation(Dialog(tab = "Advanced"));
            parameter Medium.MassFlowRate m_flow_start = 0
              "Guess value of m_flow = port_a.m_flow"
              annotation(Dialog(tab = "Advanced"));
            // Note: value of m_flow_small shall be refined by derived model,
            // based on local m_flow_nominal
            parameter Medium.MassFlowRate m_flow_small
              "Small mass flow rate for regularization of zero flow"
              annotation(Dialog(tab = "Advanced"));

            // Diagnostics
            parameter Boolean show_T = true
              "= true, if temperatures at port_a and port_b are computed"
              annotation(Dialog(tab="Advanced",group="Diagnostics"));
            parameter Boolean show_V_flow = true
              "= true, if volume flow rate at inflowing port is computed"
              annotation(Dialog(tab="Advanced",group="Diagnostics"));

            // Variables
            Medium.MassFlowRate m_flow(
               min=if allowFlowReversal then -Modelica.Constants.inf else 0,
               start = m_flow_start) "Mass flow rate in design flow direction";
            Modelica.SIunits.PressureDifference dp(start=dp_start,
                                                   displayUnit="Pa")
              "Pressure difference between port_a and port_b (= port_a.p - port_b.p)";

            Modelica.SIunits.VolumeFlowRate V_flow=
                m_flow/Modelica.Fluid.Utilities.regStep(m_flow,
                            Medium.density(
                              Medium.setState_phX(
                                p = port_a.p,
                                h = inStream(port_a.h_outflow),
                                X = inStream(port_a.Xi_outflow))),
                            Medium.density(
                                 Medium.setState_phX(
                                   p = port_b.p,
                                   h = inStream(port_b.h_outflow),
                                   X = inStream(port_b.Xi_outflow))),
                            m_flow_small) if show_V_flow
              "Volume flow rate at inflowing port (positive when flow from port_a to port_b)";

            Medium.Temperature port_a_T=
                Modelica.Fluid.Utilities.regStep(port_a.m_flow,
                            Medium.temperature(
                              Medium.setState_phX(
                                p = port_a.p,
                                h = inStream(port_a.h_outflow),
                                X = inStream(port_a.Xi_outflow))),
                            Medium.temperature(Medium.setState_phX(port_a.p, port_a.h_outflow, port_a.Xi_outflow)),
                            m_flow_small) if show_T
              "Temperature close to port_a, if show_T = true";
            Medium.Temperature port_b_T=
                Modelica.Fluid.Utilities.regStep(port_b.m_flow,
                            Medium.temperature(
                              Medium.setState_phX(
                                p = port_b.p,
                                h = inStream(port_b.h_outflow),
                                X = inStream(port_b.Xi_outflow))),
                            Medium.temperature(Medium.setState_phX(port_b.p, port_b.h_outflow, port_b.Xi_outflow)),
                            m_flow_small) if show_T
              "Temperature close to port_b, if show_T = true";
          equation
            // Pressure drop in design flow direction
            dp = port_a.p - port_b.p;

            // Design direction of mass flow rate
            m_flow = port_a.m_flow;
            assert(m_flow > -m_flow_small or allowFlowReversal,
                "Reverting flow occurs even though allowFlowReversal is false");

            // Mass balance (no storage)
            port_a.m_flow + port_b.m_flow = 0;

            // Transport of substances
            port_a.Xi_outflow = if allowFlowReversal then inStream(port_b.Xi_outflow) else Medium.X_default[1:Medium.nXi];
            port_b.Xi_outflow = inStream(port_a.Xi_outflow);

            port_a.C_outflow = if allowFlowReversal then inStream(port_b.C_outflow) else zeros(Medium.nC);
            port_b.C_outflow = inStream(port_a.C_outflow);

            annotation (
              Documentation(info="<html>
<p>
This component transports fluid between its two ports, without storing mass or energy.
Energy may be exchanged with the environment though, e.g., in the form of work.
<code>PartialTwoPortTransport</code> is intended as base class for devices like orifices, valves and simple fluid machines.</p>
<p>
Three equations need to be added by an extending class using this component:
</p>
<ul>
<li>The momentum balance specifying the relationship between the pressure drop <code>dp</code> and the mass flow rate <code>m_flow</code>,</li>
<li><code>port_b.h_outflow</code> for flow in design direction, and</li>
<li><code>port_a.h_outflow</code> for flow in reverse direction.</li>
</ul>
<p>
Moreover appropriate values shall be assigned to the following parameters:
</p>
<ul>
<li><code>dp_start</code> for a guess of the pressure drop</li>
<li><code>m_flow_small</code> for regularization of zero flow.</li>
</ul>
<h4>Implementation</h4>
<p>
This is similar to
<a href=\"modelica://Modelica.Fluid.Interfaces.PartialTwoPortTransport\">
Modelica.Fluid.Interfaces.PartialTwoPortTransport</a>
except that it does not use the <code>outer system</code> declaration.
This declaration is omitted as in building energy simulation,
many models use multiple media, an in practice,
users have not used this global definition to assign parameters.
</p>
</html>",           revisions="<html>
<ul>
<li>
September 15, 2016, by Michael Wetter:<br/>
Removed wrong annotation, which caused an error in the pedantic model check
of Dymola 2017 FD01.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/516\">#516</a>.
</li>
<li>
January 22, 2016, by Henning Francke:<br/>
Corrected type declaration of pressure.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
November 19, 2015, by Michael Wetter:<br/>
Removed assignments of parameters
<code>port_a_exposesState</code> and
<code>port_b_exposesState</code> in base class.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/351\">#351</a>.
</li>
<li>
August 15, 2015, by Filip Jorissen:<br/>
Implemented more efficient computation of <code>port_a.Xi_outflow</code>
and <code>port_a.C_outflow</code> when <code>allowFlowReversal=false</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/305\">#305</a>.
</li>
<li>
June 6, 2015, by Michael Wetter:<br/>
Removed protected conditional variables <code>state_a</code> and <code>state_b</code>,
as they were used outside of a connect statement, which causes an
error during pedantic model check in Dymola 2016.
This fixes
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/128\">#128</a>.
</li>
<li>
April 1, 2015, by Michael Wetter:<br/>
Made computation of <code>state_a</code> and <code>state_p</code>
conditional on <code>show_T</code> or <code>show_V_flow</code>.
This avoids computing temperature from enthalpy if temperature is
a state of the medium, and the result is not used.
</li>
<li>
October 21, 2014, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
October 20, 2014, by Filip Jorisson:<br/>
First implementation.
</li>
</ul>
</html>"));
          end PartialTwoPortTransport;

          partial model PartialTwoPortVector "Partial component with two ports, one of which being vectorized"

            replaceable package Medium =
              Modelica.Media.Interfaces.PartialMedium "Medium in the component"
                annotation (choices(
                  choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
                  choice(redeclare package Medium = Buildings.Media.Water "Water"),
                  choice(redeclare package Medium =
                      Buildings.Media.Antifreeze.PropyleneGlycolWater (
                    property_T=293.15,
                    X_a=0.40)
                    "Propylene glycol water, 40% mass fraction")));
            parameter Integer nPorts "Number of ports"
              annotation(Evaluate=true, Dialog(connectorSizing=true, tab="General",group="Ports"));
            parameter Boolean allowFlowReversal=true
              "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
              annotation (Dialog(tab="Assumptions"), Evaluate=true);

            Modelica.Fluid.Interfaces.FluidPort_a port_a(
              redeclare final package Medium = Medium,
              m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
              h_outflow(start=Medium.h_default, nominal=Medium.h_default))
              "Fluid connector a (positive design flow direction is from port_a to ports_b)"
              annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

            Modelica.Fluid.Interfaces.FluidPorts_b ports_b[nPorts](
              redeclare each package Medium = Medium,
              each m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
              each h_outflow(start=Medium.h_default, nominal=Medium.h_default))
              "Fluid connectors b (positive design flow direction is from port_a to ports_b)"
              annotation (Placement(transformation(extent={{90,-40},{110,40}})));

            // Diagnostics
             parameter Boolean show_T = false
              "= true, if actual temperature at port is computed"
              annotation(Dialog(tab="Advanced",group="Diagnostics"));

            Medium.ThermodynamicState sta_a=
                Medium.setState_phX(port_a.p,
                                    noEvent(actualStream(port_a.h_outflow)),
                                    noEvent(actualStream(port_a.Xi_outflow))) if
                   show_T "Medium properties in port_a";

            Medium.ThermodynamicState sta_b[nPorts]=
                Medium.setState_phX(ports_b.p,
                                    noEvent(actualStream(ports_b.h_outflow)),
                                    noEvent(actualStream(ports_b.Xi_outflow))) if
                   show_T "Medium properties in ports_b";
            annotation (
              Documentation(info="<html>
<p>
This partial model defines an interface for components with two ports,
of which one is vectorized.
</p>
<p>
The treatment of the design flow direction and of flow reversal are
determined based on the parameter <code>allowFlowReversal</code>.
The component may transport fluid and may have internal storage.
</p>
<h4>Implementation</h4>
<p>
This model is similar to
<a href=\"modelica://Modelica.Fluid.Interfaces.PartialTwoPort\">
Modelica.Fluid.Interfaces.PartialTwoPort</a>
but it does not use the <code>outer system</code> declaration.
This declaration is omitted as in building energy simulation,
many models use multiple media, and in practice,
users have not used this global definition to assign parameters.
</p>
</html>",           revisions="<html>
<ul>
<li>
January 31, 2019, by Michael Mans:<br/>
Added optional temperature state calculation as diagnostics option.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1092\">#1092</a>.
</li>
<li>
January 18, 2019, by Jianjun Hu:<br/>
Limited the media choice.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1050\">#1050</a>.
</li>
<li>
July 8, 2018, by Filip Jorissen:<br/>
Added nominal value of <code>h_outflow</code> in <code>FluidPorts</code>.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/977\">#977</a>.
</li>
<li>
November 19, 2015, by Michael Wetter:<br/>
Removed parameters
<code>port_a_exposesState</code> and
<code>port_b_exposesState</code>
for <a href=\"https://github.com/ibpsa/modelica/issues/351\">#351</a>
and
<code>showDesignFlowDirection</code>
for <a href=\"https://github.com/ibpsa/modelica/issues/349\">#349</a>.
</li>
<li>
November 13, 2015, by Michael Wetter:<br/>
Assinged <code>start</code> attribute for leaving
enthalpy at <code>port_a</code> and <code>port_b</code>.
This was done to make the model similar to
<a href=\"modelica://Buildings.Fluid.Interfaces.PartialFourPort\">
Buildings.Fluid.Interfaces.PartialFourPort</a>.
</li>
<li>
November 12, 2015, by Michael Wetter:<br/>
Removed import statement.
</li>
<li>
October 21, 2014, by Michael Wetter:<br/>
Revised implementation.
Declared medium in ports to be <code>final</code>.
</li>
<li>
October 20, 2014, by Filip Jorisson:<br/>
First implementation.
</li>
</ul>
</html>"),    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
                  graphics={
                  Polygon(
                    points={{20,-70},{60,-85},{20,-100},{20,-70}},
                    lineColor={0,128,255},
                    fillColor={0,128,255},
                    fillPattern=FillPattern.Solid,
                    visible=not allowFlowReversal),
                  Line(
                    points={{55,-85},{-60,-85}},
                    color={0,128,255},
                    visible=not allowFlowReversal),
                  Text(
                    extent={{-149,-114},{151,-154}},
                    lineColor={0,0,255},
                    textString="%name")}),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                      100}})));
          end PartialTwoPortVector;

          model StaticTwoPortConservationEquation
            "Partial model for static energy and mass conservation equations"
            extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;

            constant Boolean simplify_mWat_flow = true
              "Set to true to cause port_a.m_flow + port_b.m_flow = 0 even if mWat_flow is non-zero";

            constant Boolean prescribedHeatFlowRate = false
              "Set to true if the heat flow rate is not a function of a temperature difference to the fluid temperature";

            parameter Boolean use_mWat_flow = false
              "Set to true to enable input connector for moisture mass flow rate"
              annotation(Evaluate=true, Dialog(tab="Advanced"));

            parameter Boolean use_C_flow = false
              "Set to true to enable input connector for trace substance"
              annotation(Evaluate=true, Dialog(tab="Advanced"));

            Modelica.Blocks.Interfaces.RealInput Q_flow(unit="W")
              "Sensible plus latent heat flow rate transferred into the medium"
              annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
            Modelica.Blocks.Interfaces.RealInput mWat_flow(final quantity="MassFlowRate",
                                                           unit="kg/s") if
                 use_mWat_flow "Moisture mass flow rate added to the medium"
              annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
            Modelica.Blocks.Interfaces.RealInput[Medium.nC] C_flow if
                 use_C_flow "Trace substance mass flow rate added to the medium"
              annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));

            // Outputs that are needed in models that extend this model
            Modelica.Blocks.Interfaces.RealOutput hOut(unit="J/kg",
                                                       start=Medium.specificEnthalpy_pTX(
                                                               p=Medium.p_default,
                                                               T=Medium.T_default,
                                                               X=Medium.X_default))
              "Leaving specific enthalpy of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={-50,110}), iconTransformation(
                  extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={-50,110})));

            Modelica.Blocks.Interfaces.RealOutput XiOut[Medium.nXi](each unit="1",
                                                                    each min=0,
                                                                    each max=1)
              "Leaving species concentration of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={0,110})));
            Modelica.Blocks.Interfaces.RealOutput COut[Medium.nC](each min=0)
              "Leaving trace substances of the component"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=90,
                  origin={50,110})));

          protected
            final parameter Boolean use_m_flowInv=
              (prescribedHeatFlowRate or use_mWat_flow or use_C_flow)
              "Flag, true if m_flowInv is used in the model"
              annotation (Evaluate=true);
            final parameter Real s[Medium.nXi] = {if Modelica.Utilities.Strings.isEqual(string1=Medium.substanceNames[i],
                                                      string2="Water",
                                                      caseSensitive=false)
                                                      then 1 else 0 for i in 1:Medium.nXi}
              "Vector with zero everywhere except where species is";

            Real m_flowInv(unit="s/kg") "Regularization of 1/m_flow of port_a";

            Modelica.SIunits.MassFlowRate mXi_flow[Medium.nXi]
              "Mass flow rates of independent substances added to the medium";

            // Parameters for inverseXRegularized.
            // These are assigned here for efficiency reason.
            // Otherwise, they would need to be computed each time
            // the function is invocated.
            final parameter Real deltaReg = m_flow_small/1E3
              "Smoothing region for inverseXRegularized";

            final parameter Real deltaInvReg = 1/deltaReg
              "Inverse value of delta for inverseXRegularized";

            final parameter Real aReg = -15*deltaInvReg
              "Polynomial coefficient for inverseXRegularized";
            final parameter Real bReg = 119*deltaInvReg^2
              "Polynomial coefficient for inverseXRegularized";
            final parameter Real cReg = -361*deltaInvReg^3
              "Polynomial coefficient for inverseXRegularized";
            final parameter Real dReg = 534*deltaInvReg^4
              "Polynomial coefficient for inverseXRegularized";
            final parameter Real eReg = -380*deltaInvReg^5
              "Polynomial coefficient for inverseXRegularized";
            final parameter Real fReg = 104*deltaInvReg^6
              "Polynomial coefficient for inverseXRegularized";

            final parameter Medium.ThermodynamicState state_default = Medium.setState_pTX(
                T=Medium.T_default,
                p=Medium.p_default,
                X=Medium.X_default[1:Medium.nXi]) "Medium state at default values";
            // Density at medium default values, used to compute the size of control volumes
            final parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
              Medium.specificHeatCapacityCp(state=state_default)
              "Specific heat capacity, used to verify energy conservation";
            constant Modelica.SIunits.TemperatureDifference dTMax(min=1) = 200
              "Maximum temperature difference across the StaticTwoPortConservationEquation";
            // Conditional connectors
            Modelica.Blocks.Interfaces.RealInput mWat_flow_internal(unit="kg/s")
              "Needed to connect to conditional connector";
            Modelica.Blocks.Interfaces.RealInput C_flow_internal[Medium.nC]
              "Needed to connect to conditional connector";
          initial equation
            // Assert that the substance with name 'water' has been found.
            assert(Medium.nXi == 0 or abs(sum(s)-1) < 1e-5,
                "If Medium.nXi > 1, then substance 'water' must be present for one component.'"
                   + Medium.mediumName + "'.\n"
                   + "Check medium model.");

          equation
            // Conditional connectors
            connect(mWat_flow, mWat_flow_internal);
            if not use_mWat_flow then
              mWat_flow_internal = 0;
            end if;

            connect(C_flow, C_flow_internal);
            if not use_C_flow then
              C_flow_internal = zeros(Medium.nC);
            end if;

            // Species flow rate from connector mWat_flow
            mXi_flow = mWat_flow_internal * s;

            // Regularization of m_flow around the origin to avoid a division by zero
            // m_flowInv is only used if prescribedHeatFlowRate == true, or
            // if the input connectors mWat_flow or C_flow are enabled.
            if use_m_flowInv then
              m_flowInv = Buildings.Utilities.Math.Functions.inverseXRegularized(
                x=port_a.m_flow,
                delta=deltaReg,
                deltaInv=deltaInvReg,
                a=aReg,
                b=bReg,
                c=cReg,
                d=dReg,
                e=eReg,
                f=fReg);
            else
              // m_flowInv is not used.
              m_flowInv = 0;
            end if;

            if prescribedHeatFlowRate then
              assert(noEvent( abs(Q_flow) < dTMax*cp_default*max(m_flow_small/1E3, abs(m_flow))),
             "In " + getInstanceName() + ":
   The heat flow rate equals "           + String(Q_flow) +
             " W and the mass flow rate equals " + String(m_flow) + " kg/s,
   which results in a temperature difference "           +
             String(abs(Q_flow)/ (cp_default*max(m_flow_small/1E3, abs(m_flow)))) +
             " K > dTMax=" +String(dTMax) + " K.
   This may indicate that energy is not conserved for small mass flow rates.
   The implementation may require prescribedHeatFlowRate = false.");
            end if;

            if allowFlowReversal then
              // Formulate hOut using spliceFunction. This avoids an event iteration.
              // The introduced error is small because deltax=m_flow_small/1e3
              hOut = Buildings.Utilities.Math.Functions.regStep(
                y1=port_b.h_outflow,
                y2=port_a.h_outflow,
                x=port_a.m_flow,
                x_small=m_flow_small/1E3);
              XiOut = Buildings.Utilities.Math.Functions.regStep(
                y1=port_b.Xi_outflow,
                y2=port_a.Xi_outflow,
                x=port_a.m_flow,
                x_small=m_flow_small/1E3);
              COut = Buildings.Utilities.Math.Functions.regStep(
                y1=port_b.C_outflow,
                y2=port_a.C_outflow,
                x=port_a.m_flow,
                x_small=m_flow_small/1E3);
            else
              hOut =  port_b.h_outflow;
              XiOut = port_b.Xi_outflow;
              COut =  port_b.C_outflow;
            end if;

            //////////////////////////////////////////////////////////////////////////////////////////
            // Energy balance and mass balance

              // Mass balance (no storage)
              port_a.m_flow + port_b.m_flow = if simplify_mWat_flow then 0 else -mWat_flow_internal;

              // Substance balance
              // a) forward flow
              if use_m_flowInv then
                port_b.Xi_outflow = inStream(port_a.Xi_outflow) + mXi_flow * m_flowInv;
              else // no water is added
                assert(use_mWat_flow == false, "In " + getInstanceName() + ": Wrong implementation for forward flow.");
                port_b.Xi_outflow = inStream(port_a.Xi_outflow);
              end if;

              // b) backward flow
              if allowFlowReversal then
                if use_m_flowInv then
                  port_a.Xi_outflow = inStream(port_b.Xi_outflow) - mXi_flow * m_flowInv;
                else // no water added
                  assert(use_mWat_flow == false, "In " + getInstanceName() + ": Wrong implementation for reverse flow.");
                  port_a.Xi_outflow = inStream(port_b.Xi_outflow);
                end if;
              else // no  flow reversal
                port_a.Xi_outflow = Medium.X_default[1:Medium.nXi];
              end if;

              // Energy balance.
              // This equation is approximate since m_flow = port_a.m_flow is used for the mass flow rate
              // at both ports. Since mWat_flow_internal << m_flow, the error is small.
              if prescribedHeatFlowRate then
                port_b.h_outflow = inStream(port_a.h_outflow) + Q_flow * m_flowInv;
                if allowFlowReversal then
                  port_a.h_outflow = inStream(port_b.h_outflow) - Q_flow * m_flowInv;
                else
                  port_a.h_outflow = Medium.h_default;
                end if;
              else
                // Case with prescribedHeatFlowRate == false.
                // port_b.h_outflow is known and the equation needs to be solved for Q_flow.
                // Hence, we cannot use m_flowInv as for m_flow=0, any Q_flow would satisfiy
                // Q_flow * m_flowInv = 0.
                // The same applies for port_b.Xi_outflow and mXi_flow.
                port_a.m_flow * (inStream(port_a.h_outflow) - port_b.h_outflow)     = -Q_flow;
                if allowFlowReversal then
                  port_a.m_flow * (inStream(port_b.h_outflow)  - port_a.h_outflow)  = +Q_flow;
                else
                  // When allowFlowReversal = false, the downstream enthalpy does not matter.
                  // Therefore a dummy value is used to avoid algebraic loops
                  port_a.h_outflow = Medium.h_default;
                end if;
              end if;

            // Transport of trace substances
            if use_m_flowInv and use_C_flow then
              port_b.C_outflow =  inStream(port_a.C_outflow) + C_flow_internal * m_flowInv;
            else // no trace substance added.
              assert(not use_C_flow, "In " + getInstanceName() + ": Wrong implementation of trace substance balance for forward flow.");
              port_b.C_outflow =  inStream(port_a.C_outflow);
            end if;

            if allowFlowReversal then
              if use_C_flow then
                port_a.C_outflow = inStream(port_b.C_outflow) - C_flow_internal * m_flowInv;
              else
                port_a.C_outflow = inStream(port_b.C_outflow);
              end if;
            else
              port_a.C_outflow = zeros(Medium.nC);
            end if;

            ////////////////////////////////////////////////////////////////////////////
            // No pressure drop in this model
            port_a.p = port_b.p;

            annotation (
              preferredView="info",
              Documentation(info="<html>
<p>
This model transports fluid between its two ports, without storing mass or energy.
It implements a steady-state conservation equation for energy and mass fractions.
The model has zero pressure drop between its ports.
</p>

<h4>Typical use and important parameters</h4>
<p>
Set the parameter <code>use_mWat_flow_in=true</code> to enable an
input connector for <code>mWat_flow</code>.
Otherwise, the model uses <code>mWat_flow = 0</code>.
</p>
<p>
If the constant <code>simplify_mWat_flow = true</code>, which is its default value,
then the equation
</p>
<pre>
  port_a.m_flow + port_b.m_flow = - mWat_flow;
</pre>
<p>
is simplified as
</p>
<pre>
  port_a.m_flow + port_b.m_flow = 0;
</pre>
<p>
This causes an error in the mass balance of about <i>0.5%</i>, but generally leads to
simpler equations because the pressure drop equations are then decoupled from the
mass exchange in this component.
</p>

<p>
To increase the numerical robustness of the model, the constant
<code>prescribedHeatFlowRate</code> can be set.
Use the following settings:
</p>
<ul>
<li>Set <code>prescribedHeatFlowRate=true</code> if the <i>only</i> means of heat transfer
at the <code>heatPort</code> is a prescribed heat flow rate that
is <i>not</i> a function of the temperature difference
between the medium and an ambient temperature. Examples include an ideal electrical heater,
a pump that rejects heat into the fluid stream, or a chiller that removes heat based on a performance curve.
If the <code>heatPort</code> is not connected, then set <code>prescribedHeatFlowRate=true</code> as
in this case, <code>heatPort.Q_flow=0</code>.
</li>
<li>Set <code>prescribedHeatFlowRate=false</code> if there is heat flow at the <code>heatPort</code>
computed as <i>K * (T-heatPort.T)</i>, for some temperature <i>T</i> and some conductance <i>K</i>,
which may itself be a function of temperature or mass flow rate.<br/>
If there is a combination of <i>K * (T-heatPort.T)</i> and a prescribed heat flow rate,
for example a solar collector that dissipates heat to the ambient and receives heat from
the solar radiation, then set <code>prescribedHeatFlowRate=false</code>.
</li>
</ul>
<p>
If <code>prescribedHeatFlow=true</code>, then energy and mass balance
equations are formulated to guard against numerical problems near
zero flow that can occur if <code>Q_flow</code> or <code>m_flow</code>
are the results of an iterative solver.
</p>
<h4>Implementation</h4>
<p>
Input connectors of the model are
</p>
<ul>
<li>
<code>Q_flow</code>, which is the sensible plus latent heat flow rate added to the medium,
</li>
<li>
<code>mWat_flow</code>, which is the moisture mass flow rate added to the medium, and
</li>
<li>
<code>C_flow</code>, which is the trace substance mass flow rate added to the medium.
</li>
</ul>

<p>
The model can only be used as a steady-state model with two fluid ports.
For a model with a dynamic balance, and more fluid ports, use
<a href=\"modelica://Buildings.Fluid.Interfaces.ConservationEquation\">
Buildings.Fluid.Interfaces.ConservationEquation</a>.
</p>
</html>", revisions="<html>
<ul>
<li>
February 12, 2019, by Filip Jorissen:<br/>
Removed obsolete division by <code>TMax</code> in assert.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1097\">#1097</a>.
</li>
<li>
June 23, 2018, by Filip Jorissen:<br/>
Added more details to energy conservation assert to facilitate
debugging.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/962\">#962</a>.
</li>
<li>
March 30, 2018, by Filip Jorissen:<br/>
Added <code>getInstanceName()</code> in asserts to facilitate
debugging.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/901\">#901</a>.
</li>
<li>
April 24, 2017, by Michael Wetter and Filip Jorissen:<br/>
Reimplemented check for energy conversion.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/741\">#741</a>.
</li>
<li>
April 24, 2017, by Michael Wetter:<br/>
Reverted change from April 21, 2017.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/741\">#741</a>.
</li>
<li>
April 21, 2017, by Filip Jorissen:<br/>
Revised test for energy conservation at small mass flow rates.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/741\">#741</a>.
</li>
<li>
October 23, 2016, by Filip Jorissen:<br/>
Added test for energy conservation at small mass flow rates.
</li>
<li>
March 17, 2016, by Michael Wetter:<br/>
Refactored model and implmented <code>regStep</code> instead of <code>spliceFunction</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/247\">#247</a>
and for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/300\">#300</a>.
</li>
<li>
September 3, 2015, by Filip Jorissen:<br/>
Revised implementation of conservation of vapor mass.
Added new variable <code>mFlow_inv_b</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/247\">#247</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Removed <code>constant sensibleOnly</code> as this is no longer used because
the model uses <code>use_mWat_flow</code>.<br/>
Changed condition that determines whether <code>m_flowInv</code> needs to be
computed because the change from January 20 introduced an error in
<a href=\"modelica://Buildings.Fluid.MassExchangers.Examples.ConstantEffectiveness\">
Buildings.Fluid.MassExchangers.Examples.ConstantEffectiveness</a>.
</li>
<li>
January 20, 2016, by Filip Jorissen:<br/>
Removed if-else block in code for parameter <code>sensibleOnly</code>
since this is no longer needed to simplify the equations.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/372\">#372</a>.
</li>
<li>
January 17, 2016, by Michael Wetter:<br/>
Added parameter <code>use_C_flow</code> and converted <code>C_flow</code>
to a conditionally removed connector.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/372\">#372</a>.
</li>
<li>
December 16, 2015, by Michael Wetter:<br/>
Removed the units of <code>C_flow</code> to allow for PPM.
</li>
<li>
December 2, 2015, by Filip Jorissen:<br/>
Added input <code>C_flow</code> and code for handling trace substance insertions.
November 19, 2015, by Michael Wetter:<br/>
Removed assignment of parameter
<code>showDesignFlowDirection</code> in <code>extends</code> statement.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/349\">#349</a>.
</li>
<li>
September 14, 2015, by Filip Jorissen:<br/>
Rewrote some equations for better readability.
</li>
<li>
August 11, 2015, by Michael Wetter:<br/>
Refactored implementation of
<a href=\"modelica://Buildings.Utilities.Math.Functions.inverseXRegularized\">
Buildings.Utilities.Math.Functions.inverseXRegularized</a>
to allow function to be inlined and to factor out the computation
of arguments that only depend on parameters.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/302\">issue 302</a>.
</li>
<li>
July 17, 2015, by Michael Wetter:<br/>
Corrected bug for situation with latent heat exchange and flow reversal not
allowed.
The previous formulation was singular.
This caused some models to not translate.
The error was introduced in
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/282\">#282</a>.
</li>
<li>
July 17, 2015, by Michael Wetter:<br/>
Added constant <code>simplify_mWat_flow</code> to remove dependencies of the pressure drop
calculation on the moisture balance.
</li>
<li>
July 2, 2015 by Michael Wetter:<br/>
Revised implementation of conservation equations,
added default values for outlet quantities at <code>port_a</code>
if <code>allowFlowReversal=false</code> and
updated documentation.
See
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/281\">
issue 281</a> for a discussion.
</li>
<li>
July 1, 2015, by Filip Jorissen:<br/>
Revised implementation so that equations are always consistent
and do not lead to division by zero,
also when connecting a <code>prescribedHeatFlowRate</code>
to <code>MixingVolume</code> instances.
Renamed <code>use_safeDivision</code> into <code>prescribedHeatFlowRate</code>.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/282\">#282</a>
for a discussion.
</li>
<li>
May 6, 2015, by Michael Wetter:<br/>
Corrected documentation.
</li>
<li>
February 11, 2014 by Michael Wetter:<br/>
Improved documentation for <code>Q_flow</code> input.
</li>
<li>
October 21, 2013 by Michael Wetter:<br/>
Corrected sign error in the equation that is used if <code>use_safeDivision=false</code>
and <code>sensibleOnly=true</code>.
This only affects internal numerical tests, but not any examples in the library
as the constant <code>use_safeDivision</code> is set to <code>true</code> by default.
</li>
<li>
September 25, 2013 by Michael Wetter:<br/>
Reformulated computation of outlet properties to avoid an event at zero mass flow rate.
</li>
<li>
September 17, 2013 by Michael Wetter:<br/>
Added start value for <code>hOut</code>.
</li>
<li>September 10, 2013 by Michael Wetter:<br/>
Removed unrequired parameter <code>i_w</code>.
</li>
<li>
May 7, 2013 by Michael Wetter:<br/>
Removed <code>for</code> loops for species balance and trace substance balance,
as they cause the error <code>Error: Operand port_a.Xi_outflow[1] to operator inStream is not a stream variable.</code>
in OpenModelica.
</li>
<li>
March 27, 2013 by Michael Wetter:<br/>
Removed wrong unit attribute of <code>COut</code>,
and added min and max attributes for <code>XiOut</code>.
</li>
<li>
June 22, 2012 by Michael Wetter:<br/>
Reformulated implementation with <code>m_flowInv</code> to use <code>port_a.m_flow * ...</code>
if <code>use_safeDivision=false</code>. This avoids a division by zero if
<code>port_a.m_flow=0</code>.
</li>
<li>
February 7, 2012 by Michael Wetter:<br/>
Revised base classes for conservation equations in <code>Buildings.Fluid.Interfaces</code>.
</li>
<li>
December 14, 2011 by Michael Wetter:<br/>
Changed assignment of <code>hOut</code>, <code>XiOut</code> and
<code>COut</code> to no longer declare that it is continuous.
The declaration of continuity, i.e, the
<code>smooth(0, if (port_a.m_flow >= 0) then ...)</code> declaration,
was required for Dymola 2012 to simulate, but it is no longer needed
for Dymola 2012 FD01.
</li>
<li>
August 19, 2011, by Michael Wetter:<br/>
Changed assignment of <code>hOut</code>, <code>XiOut</code> and
<code>COut</code> to declare that it is not differentiable.
</li>
<li>
August 4, 2011, by Michael Wetter:<br/>
Moved linearized pressure drop equation from the function body to the equation
section. With the previous implementation,
the symbolic processor may not rearrange the equations, which can lead
to coupled equations instead of an explicit solution.
</li>
<li>
March 29, 2011, by Michael Wetter:<br/>
Changed energy and mass balance to avoid a division by zero if <code>m_flow=0</code>.
</li>
<li>
March 27, 2011, by Michael Wetter:<br/>
Added <code>homotopy</code> operator.
</li>
<li>
August 19, 2010, by Michael Wetter:<br/>
Fixed bug in energy and moisture balance that affected results if a component
adds or removes moisture to the air stream.
In the old implementation, the enthalpy and species
outflow at <code>port_b</code> was multiplied with the mass flow rate at
<code>port_a</code>. The old implementation led to small errors that were proportional
to the amount of moisture change. For example, if the moisture added by the component
was <code>0.005 kg/kg</code>, then the error was <code>0.5%</code>.
Also, the results for forward flow and reverse flow differed by this amount.
With the new implementation, the energy and moisture balance is exact.
</li>
<li>
March 22, 2010, by Michael Wetter:<br/>
Added constant <code>sensibleOnly</code> to
simplify species balance equation.
</li>
<li>
April 10, 2009, by Michael Wetter:<br/>
Added model to compute flow friction.
</li>
<li>
April 22, 2008, by Michael Wetter:<br/>
Revised to add mass balance.
</li>
<li>
March 17, 2008, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),    Icon(coordinateSystem(
                  preserveAspectRatio=true,
                  extent={{-100,-100},{100,100}},
                  grid={1,1}), graphics={Rectangle(
                    extent={{-100,100},{100,-100}},
                    fillColor={135,135,135},
                    fillPattern=FillPattern.Solid,
                    pattern=LinePattern.None),
                  Text(
                    extent={{-93,72},{-58,89}},
                    lineColor={0,0,127},
                    textString="Q_flow"),
                  Text(
                    extent={{-93,37},{-58,54}},
                    lineColor={0,0,127},
                    textString="mWat_flow"),
                  Text(
                    extent={{-41,103},{-10,117}},
                    lineColor={0,0,127},
                    textString="hOut"),
                  Text(
                    extent={{10,103},{41,117}},
                    lineColor={0,0,127},
                    textString="XiOut"),
                  Text(
                    extent={{61,103},{92,117}},
                    lineColor={0,0,127},
                    textString="COut"),
                  Line(points={{-42,55},{-42,-84}}, color={255,255,255}),
                  Polygon(
                    points={{-42,67},{-50,45},{-34,45},{-42,67}},
                    lineColor={255,255,255},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid),
                  Polygon(
                    points={{87,-73},{65,-65},{65,-81},{87,-73}},
                    lineColor={255,255,255},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid),
                  Line(points={{-56,-73},{81,-73}}, color={255,255,255}),
                  Line(points={{6,14},{6,-37}},     color={255,255,255}),
                  Line(points={{54,14},{6,14}},     color={255,255,255}),
                  Line(points={{6,-37},{-42,-37}},  color={255,255,255})}));
          end StaticTwoPortConservationEquation;

          record LumpedVolumeDeclarations "Declarations for lumped volumes"
            replaceable package Medium =
              Modelica.Media.Interfaces.PartialMedium "Medium in the component"
                annotation (choices(
                  choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
                  choice(redeclare package Medium = Buildings.Media.Water "Water"),
                  choice(redeclare package Medium =
                      Buildings.Media.Antifreeze.PropyleneGlycolWater (
                        property_T=293.15,
                        X_a=0.40)
                        "Propylene glycol water, 40% mass fraction")));

            // Assumptions
            parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
              "Type of energy balance: dynamic (3 initialization options) or steady state"
              annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
            parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
              "Type of mass balance: dynamic (3 initialization options) or steady state"
              annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
            final parameter Modelica.Fluid.Types.Dynamics substanceDynamics=energyDynamics
              "Type of independent mass fraction balance: dynamic (3 initialization options) or steady state"
              annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
            final parameter Modelica.Fluid.Types.Dynamics traceDynamics=energyDynamics
              "Type of trace substance balance: dynamic (3 initialization options) or steady state"
              annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

            // Initialization
            parameter Medium.AbsolutePressure p_start = Medium.p_default
              "Start value of pressure"
              annotation(Dialog(tab = "Initialization"));
            parameter Medium.Temperature T_start=Medium.T_default
              "Start value of temperature"
              annotation(Dialog(tab = "Initialization"));
            parameter Medium.MassFraction X_start[Medium.nX](
                 quantity=Medium.substanceNames) = Medium.X_default
              "Start value of mass fractions m_i/m"
              annotation (Dialog(tab="Initialization", enable=Medium.nXi > 0));
            parameter Medium.ExtraProperty C_start[Medium.nC](
                 quantity=Medium.extraPropertiesNames)=fill(0, Medium.nC)
              "Start value of trace substances"
              annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));
            parameter Medium.ExtraProperty C_nominal[Medium.nC](
                 quantity=Medium.extraPropertiesNames) = fill(1E-2, Medium.nC)
              "Nominal value of trace substances. (Set to typical order of magnitude.)"
             annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));
            parameter Real mSenFac(min=1)=1
              "Factor for scaling the sensible thermal mass of the volume"
              annotation(Dialog(tab="Dynamics"));

          annotation (preferredView="info",
          Documentation(info="<html>
<p>
This class contains parameters and medium properties
that are used in the lumped  volume model, and in models that extend the
lumped volume model.
</p>
<p>
These parameters are used for example by
<a href=\"modelica://Buildings.Fluid.Interfaces.ConservationEquation\">
Buildings.Fluid.Interfaces.ConservationEquation</a>,
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolume\">
Buildings.Fluid.MixingVolumes.MixingVolume</a> and
<a href=\"modelica://Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2\">
Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2</a>.
</p>
</html>", revisions="<html>
<ul>
<li>
January 18, 2019, by Jianjun Hu:<br/>
Limited the media choice.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1050\">#1050</a>.
</li>
<li>
November 9, 2018 by Michael Wetter:<br/>
Limited choices of media that are displayed in the pull down menu of
graphical editors.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1050\">issue 1050</a>.
</li>
<li>
April 11, 2016 by Michael Wetter:<br/>
Corrected wrong hyperlink in documentation for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/450\">issue 450</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Added <code>quantity=Medium.substanceNames</code> for <code>X_start</code>.
</li>
<li>
October 21, 2014, by Filip Jorissen:<br/>
Added parameter <code>mFactor</code> to increase the thermal capacity.
</li>
<li>
August 2, 2011, by Michael Wetter:<br/>
Set <code>substanceDynamics</code> and <code>traceDynamics</code> to final
and equal to <code>energyDynamics</code>,
as there is no need to make them different from <code>energyDynamics</code>.
</li>
<li>
August 1, 2011, by Michael Wetter:<br/>
Changed default value for <code>energyDynamics</code> to
<code>Modelica.Fluid.Types.Dynamics.DynamicFreeInitial</code> because
<code>Modelica.Fluid.Types.Dynamics.SteadyStateInitial</code> leads
to high order DAE that Dymola cannot reduce.
</li>
<li>
July 31, 2011, by Michael Wetter:<br/>
Changed default value for <code>energyDynamics</code> to
<code>Modelica.Fluid.Types.Dynamics.SteadyStateInitial</code>.
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
          end LumpedVolumeDeclarations;
        annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains basic classes that are used to build
component models that change the state of the
fluid. The classes are not directly usable, but can
be extended when building a new model.
</p>
</html>"));
        end Interfaces;

        package BaseClasses "Package with base classes for Buildings.Fluid"
          extends Modelica.Icons.BasesPackage;

          partial model PartialResistance "Partial model for a hydraulic resistance"
              extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
              show_T=false,
              dp(nominal=if dp_nominal_pos > Modelica.Constants.eps then dp_nominal_pos else 1),
              m_flow(nominal=if m_flow_nominal_pos > Modelica.Constants.eps then m_flow_nominal_pos else 1),
              final m_flow_small=1E-4*abs(m_flow_nominal));

            constant Boolean homotopyInitialization = true "= true, use homotopy method"
              annotation(HideResult=true);

            parameter Boolean from_dp = false
              "= true, use m_flow = f(dp) else dp = f(m_flow)"
              annotation (Evaluate=true, Dialog(tab="Advanced"));

            parameter Modelica.SIunits.PressureDifference dp_nominal(displayUnit="Pa")
              "Pressure drop at nominal mass flow rate"
              annotation(Dialog(group = "Nominal condition"));

            parameter Boolean linearized = false
              "= true, use linear relation between m_flow and dp for any flow rate"
              annotation(Evaluate=true, Dialog(tab="Advanced"));

            parameter Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
              "Turbulent flow if |m_flow| >= m_flow_turbulent";

          protected
            parameter Medium.ThermodynamicState sta_default=
               Medium.setState_pTX(T=Medium.T_default, p=Medium.p_default, X=Medium.X_default);
            parameter Modelica.SIunits.DynamicViscosity eta_default=Medium.dynamicViscosity(sta_default)
              "Dynamic viscosity, used to compute transition to turbulent flow regime";

            final parameter Modelica.SIunits.MassFlowRate m_flow_nominal_pos = abs(m_flow_nominal)
              "Absolute value of nominal flow rate";
            final parameter Modelica.SIunits.PressureDifference dp_nominal_pos(displayUnit="Pa") = abs(dp_nominal)
              "Absolute value of nominal pressure difference";
          initial equation
            assert(homotopyInitialization, "In " + getInstanceName() +
              ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
              level = AssertionLevel.warning);

          equation
            // Isenthalpic state transformation (no storage and no loss of energy)
            port_a.h_outflow = if allowFlowReversal then inStream(port_b.h_outflow) else Medium.h_default;
            port_b.h_outflow = inStream(port_a.h_outflow);

            // Mass balance (no storage)
            port_a.m_flow + port_b.m_flow = 0;

            // Transport of substances
            port_a.Xi_outflow = if allowFlowReversal then inStream(port_b.Xi_outflow) else Medium.X_default[1:Medium.nXi];
            port_b.Xi_outflow = inStream(port_a.Xi_outflow);

            port_a.C_outflow = if allowFlowReversal then inStream(port_b.C_outflow) else zeros(Medium.nC);
            port_b.C_outflow = inStream(port_a.C_outflow);

            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                      -100},{100,100}}), graphics={
                  Rectangle(
                    extent={{-100,40},{100,-40}},
                    lineColor={0,0,0},
                    fillPattern=FillPattern.HorizontalCylinder,
                    fillColor={192,192,192}),
                  Rectangle(
                    extent={{-100,22},{100,-24}},
                    lineColor={0,0,0},
                    fillPattern=FillPattern.HorizontalCylinder,
                    fillColor={0,127,255}),
                  Rectangle(
                    visible=linearized,
                    extent={{-100,22},{100,-22}},
                    fillPattern=FillPattern.Backward,
                    fillColor={0,128,255},
                    pattern=LinePattern.None,
                    lineColor={255,255,255}),
                  Rectangle(
                    extent=DynamicSelect({{-100,10},{-100,10}}, {{100,10},{100+200*max(-1, min(0, m_flow/(abs(m_flow_nominal)))),-10}}),
                    lineColor={28,108,200},
                    fillColor={255,0,0},
                    fillPattern=FillPattern.Solid,
                    pattern=LinePattern.None),
                  Rectangle(
                    extent=DynamicSelect({{-100,10},{-100,10}}, {{-100,10},{-100+200*min(1, max(0, m_flow/abs(m_flow_nominal))),-10}}),
                    lineColor={28,108,200},
                    fillColor={0,0,0},
                    fillPattern=FillPattern.Solid,
                    pattern=LinePattern.None)}),
                    defaultComponentName="res",
          Documentation(info="<html>
<p>
Partial model for a flow resistance, possible with variable flow coefficient.
Models that extend this class need to implement an equation that relates
<code>m_flow</code> and <code>dp</code>, and they need to assign the parameter
<code>m_flow_turbulent</code>.
</p>
<p>
See for example
<a href=\"modelica://Buildings.Fluid.FixedResistances.PressureDrop\">
Buildings.Fluid.FixedResistances.PressureDrop</a> for a model that extends
this base class.
</p>
</html>",           revisions="<html>
<ul>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">Buildings, #1341</a>.
</li>
<li>
February 26, 2020, by Michael Wetter:<br/>
Changed icon to display its operating state.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1294\">#1294</a>.
</li>
<li>
October 25, 2019, by Jianjun Hu:<br/>
Improved icon graphics annotation. This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1225\">#1225</a>.
</li>
<li>
November 3, 2016, by Michael Wetter:<br/>
Removed start value for pressure difference
to simplify the parameter window.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/552\">#552</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Avoided assignment of <code>dp(nominal=0)</code> if <code>dp_nominal_pos = 0</code>
and of <code>m_flow(nominal=0)</code> if <code>m_flow_nominal_pos = 0</code>
as nominal values are not allowed to be zero.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
August 15, 2015, by Filip Jorissen:<br/>
Implemented more efficient computation of <code>port_a.Xi_outflow</code>,
<code>port_a.h_outflow</code>
and <code>port_a.C_outflow</code> when <code>allowFlowReversal=false</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/281\">#281</a>.
</li>
<li>
January 13, 2015, by Marcus Fuchs:<br/>
Revised revisions section (there were two revisions statements)
</li>
<li>
November 20, 2014 by Michael Wetter:<br/>
Removed <code>start</code> attribute for <code>m_flow</code>
as this is already set in its base class.
</li>
<li>
October 8, 2013 by Michael Wetter:<br/>
Removed propagation of <code>show_V_flow</code>
to base class as it has no longer this parameter.
</li>
<li>
December 14, 2012 by Michael Wetter:<br/>
Renamed protected parameters for consistency with the naming conventions.
</li>
<li>
February 12, 2012, by Michael Wetter:<br/>
Removed duplicate declaration of <code>m_flow_nominal</code>.
</li>
<li>
February 3, 2012, by Michael Wetter:<br/>
Made assignment of <code>m_flow_small</code> <code>final</code> as it is no
longer used in the base class.
</li>
<li>
January 16, 2012, by Michael Wetter:<br/>
To simplify object inheritance tree, revised base classes
<code>Buildings.Fluid.BaseClasses.PartialResistance</code>,
<code>Buildings.Fluid.Actuators.BaseClasses.PartialTwoWayValve</code>,
<code>Buildings.Fluid.Actuators.BaseClasses.PartialDamperExponential</code>,
<code>Buildings.Fluid.Actuators.BaseClasses.PartialActuator</code>
and model
<code>Buildings.Fluid.FixedResistances.PressureDrop</code>.
</li>
<li>
August 5, 2011, by Michael Wetter:<br/>
Moved linearized pressure drop equation from the function body to the equation
section. With the previous implementation,
the symbolic processor may not rearrange the equations, which can lead
to coupled equations instead of an explicit solution.
</li>
<li>
June 20, 2011, by Michael Wetter:<br/>
Set start values for <code>m_flow</code> and <code>dp</code> to zero, since
most HVAC systems start at zero flow. With this change, the start values
appear in the GUI and can be set by the user.
</li>
<li>
April 2, 2011 by Michael Wetter:<br/>
Added <code>m_flow_nominal_pos</code> and <code>dp_nominal_pos</code> to allow
providing negative nominal values which will be used, for example, to set start
values of flow splitters which may have negative flow rates and pressure drop
at the initial condition.
</li>
<li>
March 27, 2011, by Michael Wetter:<br/>
Added <code>homotopy</code> operator.
</li>
<li>
March 23, 2011 by Michael Wetter:<br/>
Added homotopy operator.
</li>
<li>
March 30, 2010 by Michael Wetter:<br/>
Changed base classes to allow easier initialization.
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
Extracted pressure drop computation and implemented it in the
new model
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.BasicFlowModel\">
Buildings.Fluid.BaseClasses.FlowModels.BasicFlowModel</a>.
</li>
<li>
September 18, 2008, by Michael Wetter:<br/>
Added equations for the mass balance of extra species flow,
i.e., <code>C</code> and <code>mC_flow</code>.
</li>
<li>
July 20, 2007 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
          end PartialResistance;

          package FlowModels "Flow models for pressure drop calculations"
            extends Modelica.Icons.BasesPackage;

            function basicFlowFunction_dp
              "Function that computes mass flow rate for given pressure drop"

              input Modelica.SIunits.PressureDifference dp(displayUnit="Pa")
                "Pressure difference between port_a and port_b (= port_a.p - port_b.p)";
              input Real k(min=0, unit="")
                "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
              input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
                "Mass flow rate where transition to turbulent flow occurs";
              output Modelica.SIunits.MassFlowRate m_flow
                "Mass flow rate in design flow direction";
            protected
              Modelica.SIunits.PressureDifference dp_turbulent = (m_flow_turbulent/k)^2
                "Pressure where flow changes to turbulent";
              Real dpNorm=dp/dp_turbulent
                "Normalised pressure difference";
              Real dpNormSq=dpNorm^2
                "Square of normalised pressure difference";
            algorithm
               m_flow := smooth(2, if noEvent(abs(dp)>dp_turbulent)
                           then sign(dp)*k*sqrt(abs(dp))
                           else (1.40625  + (0.15625*dpNormSq - 0.5625)*dpNormSq)*m_flow_turbulent*dpNorm);
              annotation(Inline=false,
                       smoothOrder=2,
                       derivative(order=1, zeroDerivative=k, zeroDerivative=m_flow_turbulent)=
                         Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp_der,
                       inverse(dp=Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow(
                         m_flow=m_flow, k=k, m_flow_turbulent=m_flow_turbulent)),
                       Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                        {100,100}}), graphics={Line(
                      points={{-80,-40},{-80,60},{80,-40},{80,60}},
                      color={0,0,255},
                      thickness=1), Text(
                      extent={{-40,-40},{40,-80}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.Sphere,
                      fillColor={232,0,0},
                      textString="%name")}),
            Documentation(info="<html>
<p>
Function that computes the pressure drop of flow elements as
</p>
<p align=\"center\" style=\"font-style:italic;\">
  m = sign(&Delta;p) k  &radic;<span style=\"text-decoration:overline;\">&nbsp;&Delta;p &nbsp;</span>
</p>
<p>
with regularization near the origin.
Therefore, the flow coefficient is
</p>
<p align=\"center\" style=\"font-style:italic;\">
  k = m &frasl; &radic;<span style=\"text-decoration:overline;\">&nbsp;&Delta;p &nbsp;</span>
</p>
<p>
The input <code>m_flow_turbulent</code> determines the location of the regularization.
</p>
</html>",             revisions="<html>
<ul>
<li>
November 9, 2019, by Filip Jorissen:<br/>
Added <code>smooth(2, . )</code> for avoiding
a warning in the check valve model.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/pull/1240\">#1240</a>.
</li>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">basicFlowFunction_dp</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
March 19, 2016, by Michael Wetter:<br/>
Added <code>abs</code> function for
<code>Buildings.Fluid.FixedResistances.Validation.PressureDropsExplicit</code>
to work in OpenModelica.
See <a href=\"https://trac.openmodelica.org/OpenModelica/ticket/3778\">
OpenModelica ticket 3778</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 28, 2015, by Michael Wetter:<br/>
Removed double declaration of <code>smooth(..)</code> and <code>smoothOrder</code>
and changed <code>Inline=true</code> to <code>LateInline=true</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/301\">issue 301</a>.
</li>
<li>
July 15, 2015, by Filip Jorissen:<br/>
New, more efficient implementation based on regularisation using simple polynomial.
Expanded common subexpressions for function inlining to be possible.
Set <code>Inline=true</code> for inlining to occur.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/279\">#279</a>.
</li>
<li>
January 9, 2014, by Michael Wetter:<br/>
Correct revision section, of which there were two.
</li>
<li>
August 10, 2011, by Michael Wetter:<br/>
Removed <code>if-then</code> optimization that set <code>m_flow=0</code> if <code>dp=0</code>,
as this causes the derivative to be discontinuous at <code>dp=0</code>.
</li>
<li>
August 4, 2011, by Michael Wetter:<br/>
Removed option to use a linear function. The linear implementation is now done
in models that call this function. With the previous implementation,
the symbolic processor may not rearrange the equations, which can lead
to coupled equations instead of an explicit solution.
</li>
<li>
March 29, 2010 by Michael Wetter:<br/>
Changed implementation to allow <code>k=0</code>, which is
the case for a closed valve with no leakage
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
            end basicFlowFunction_dp;

            function basicFlowFunction_dp_der
              "1st derivative of function that computes mass flow rate for given pressure drop"
              extends Modelica.Icons.Function;

              input Modelica.SIunits.PressureDifference dp(displayUnit="Pa")
                "Pressure difference between port_a and port_b (= port_a.p - port_b.p)";
              input Real k(min=0, unit="")
                "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
              input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
                "Mass flow rate where transition to turbulent flow occurs";
              input Real dp_der
                "Derivative of pressure difference between port_a and port_b (= port_a.p - port_b.p)";
              output Real m_flow_der(unit="kg/s2")
                "Derivative of mass flow rate in design flow direction";
            protected
              Modelica.SIunits.PressureDifference dp_turbulent = (m_flow_turbulent/k)^2
                "Pressure where flow changes to turbulent";
              Real dpNormSq=(dp/dp_turbulent)^2
                "Square of normalised pressure difference";
            algorithm
             m_flow_der := (if noEvent(abs(dp)>dp_turbulent)
                            then 0.5*k/sqrt(abs(dp))
                            else (1.40625  + (0.78125*dpNormSq - 1.6875)*dpNormSq)*m_flow_turbulent/dp_turbulent)*dp_der;
             annotation (Inline=false,
                         smoothOrder=1,
                         derivative(order=2, zeroDerivative=k, zeroDerivative=m_flow_turbulent)=
                           Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp_der2,
            Documentation(info="<html>
<p>
Function that implements the first order derivative of
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>
with respect to the mass flow rate.
</p>
</html>",   revisions="<html>
<ul>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
April 14, 2017, by Filip Jorissen:<br/>
Changed implementation such that it cannot lead to square roots
of negative numbers and reduced the number of required operations.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/723\">#723</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 29, 2015, by Michael Wetter:<br/>
First implementation to avoid in Dymola 2016 the warning
\"Differentiating ... under the assumption that it is continuous at switching\".
</li>
</ul>
</html>"));
            end basicFlowFunction_dp_der;

            function basicFlowFunction_dp_der2
              "2nd derivative of flow function2nd derivative of function that computes mass flow rate for given pressure drop"
              extends Modelica.Icons.Function;

              input Modelica.SIunits.PressureDifference dp(displayUnit="Pa")
                "Pressure difference between port_a and port_b (= port_a.p - port_b.p)";
              input Real k(min=0, unit="")
                "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
              input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
                "Mass flow rate where transition to turbulent flow occurs";
              input Real dp_der
                "1st derivative of pressure difference between port_a and port_b (= port_a.p - port_b.p)";
              input Real dp_der2
                "2nd derivative of pressure difference between port_a and port_b (= port_a.p - port_b.p)";
              output Real m_flow_der2
                "2nd derivative of mass flow rate in design flow direction";
            protected
              Modelica.SIunits.PressureDifference dp_turbulent = (m_flow_turbulent/k)^2
                "Pressure where flow changes to turbulent";
              Real dpNorm=dp/dp_turbulent
                "Normalised pressure difference";
              Real dpNormSq=dpNorm^2
                "Square of normalised pressure difference";
            algorithm
             m_flow_der2 := if noEvent(abs(dp)>dp_turbulent)
                             then 0.5*k/sqrt(abs(dp))*(-0.5/dp * dp_der^2 + dp_der2)
                             else m_flow_turbulent/dp_turbulent*(
                                   (1.40625  + (0.78125*dpNormSq - 1.6875)*dpNormSq)*dp_der2
                                 + (-3.375 + 3.125*dpNormSq)*dpNorm/dp_turbulent*dp_der^2);

             annotation (smoothOrder=0,
             Inline=false,
            Documentation(info="<html>
<p>
Function that implements the second order derivative of
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>
with respect to the mass flow rate.
</p>
</html>",   revisions="<html>
<ul>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 29, 2015, by Michael Wetter:<br/>
First implementation to avoid in Dymola 2016 the warning
\"Differentiating ... under the assumption that it is continuous at switching\".
</li>
</ul>
</html>"));
            end basicFlowFunction_dp_der2;

            function basicFlowFunction_m_flow
              "Function that computes pressure drop for given mass flow rate"

              input Modelica.SIunits.MassFlowRate m_flow
                "Mass flow rate in design flow direction";
              input Real k(unit="")
                "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
              input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
                "Mass flow rate where transition to turbulent flow occurs";
              output Modelica.SIunits.PressureDifference dp(displayUnit="Pa")
                "Pressure difference between port_a and port_b (= port_a.p - port_b.p)";
            protected
              Modelica.SIunits.PressureDifference dp_turbulent = (m_flow_turbulent/k)^2
                "Pressure where flow changes to turbulent";
              Real m_flowNorm = m_flow/m_flow_turbulent
                "Normalised mass flow rate";
              Real m_flowNormSq = m_flowNorm^2
                "Square of normalised mass flow rate";

            algorithm
             dp := smooth(2, if noEvent(abs(m_flow)>m_flow_turbulent)
                  then sign(m_flow)*(m_flow/k)^2
                  else (0.375 + (0.75-0.125*m_flowNormSq)*m_flowNormSq)*dp_turbulent*m_flowNorm);

             annotation (Inline=false,
                         smoothOrder=2,
                         derivative(order=1, zeroDerivative=k, zeroDerivative=m_flow_turbulent)=
                           Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow_der,
                         inverse(m_flow=Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp(
                           dp=dp, k=k, m_flow_turbulent=m_flow_turbulent)),
                         Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                        -100},{100,100}}), graphics={Line(
                      points={{-80,-40},{-80,60},{80,-40},{80,60}},
                      color={0,0,255},
                      thickness=1), Text(
                      extent={{-40,-40},{40,-80}},
                      lineColor={0,0,0},
                      fillPattern=FillPattern.Sphere,
                      fillColor={232,0,0},
                      textString="%name")}),
            Documentation(info="<html>
<p>
Function that computes the pressure drop of flow elements as
</p>
<p align=\"center\" style=\"font-style:italic;\">
  &Delta;p = sign(m) (m &frasl; k)<sup>2</sup>
</p>
<p>
with regularization near the origin.
Therefore, the flow coefficient is
</p>
<p align=\"center\" style=\"font-style:italic;\">
  k = m &frasl; &radic;<span style=\"text-decoration:overline;\">&nbsp;&Delta;p &nbsp;</span>
</p>
<p>
The input <code>m_flow_turbulent</code> determines the location of the regularization.
</p>
</html>",   revisions="<html>
<ul>
<li>
December 9, 2019, by Michael Wetter:<br/>
Added <code>smooth(2, . )</code>, similar to
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/pull/1240\">#1240</a>.
</li>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 28, 2015, by Michael Wetter:<br/>
Removed double declaration of <code>smooth(..)</code> and <code>smoothOrder</code>
and changed <code>Inline=true</code> to <code>LateInline=true</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/301\">issue 301</a>.
</li>
<li>
July 15, 2015, by Filip Jorissen:<br/>
New, more efficient implementation based on regularisation using simple polynomial.
Expanded common subexpressions for function inlining to be possible.
Set <code>Inline=true</code> for inlining to occur.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/279\">#279</a>.
</li>
<li>
August 10, 2011, by Michael Wetter:<br/>
Removed <code>if-then</code> optimization that set <code>dp=0</code> if <code>m_flow=0</code>,
as this causes the derivative to be discontinuous at <code>m_flow=0</code>.
</li>
<li>
August 4, 2011, by Michael Wetter:<br/>
Removed option to use a linear function. The linear implementation is now done
in models that call this function. With the previous implementation,
the symbolic processor may not rearrange the equations, which can lead
to coupled equations instead of an explicit solution.
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
            end basicFlowFunction_m_flow;

            function basicFlowFunction_m_flow_der
              "1st derivative of function that computes pressure drop for given mass flow rate"
              extends Modelica.Icons.Function;

              input Modelica.SIunits.MassFlowRate m_flow
                "Mass flow rate in design flow direction";
              input Real k(unit="")
                "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
              input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
                "Mass flow rate where transition to turbulent flow occurs";
              input Real m_flow_der(unit="kg/s2")
                "Derivative of mass flow rate in design flow direction";
              output Real dp_der
                "Derivative of pressure difference between port_a and port_b (= port_a.p - port_b.p)";
            protected
              Modelica.SIunits.PressureDifference dp_turbulent = (m_flow_turbulent/k)^2
                "Pressure where flow changes to turbulent";
              Real m_flowNormSq = (m_flow/m_flow_turbulent)^2
                "Square of normalised mass flow rate";
            algorithm
             dp_der :=(if noEvent(abs(m_flow)>m_flow_turbulent)
                       then sign(m_flow)*2*m_flow/k^2
                       else (0.375  + (2.25 - 0.625*m_flowNormSq)*m_flowNormSq)*dp_turbulent/m_flow_turbulent)*m_flow_der;

             annotation (Inline=false,
                         smoothOrder=1,
                         derivative(order=2, zeroDerivative=k, zeroDerivative=m_flow_turbulent)=
                         Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow_der2,
            Documentation(info="<html>
<p>
Function that implements the first order derivative of
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow</a>
with respect to the mass flow rate.
</p>
</html>",   revisions="<html>
<ul>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
July 29, 2015, by Michael Wetter:<br/>
First implementation to avoid in Dymola 2016 the warning
\"Differentiating ... under the assumption that it is continuous at switching\".
</li>
</ul>
</html>"));
            end basicFlowFunction_m_flow_der;

            function basicFlowFunction_m_flow_der2
              "2nd derivative of function that computes pressure drop for given mass flow rate"
              extends Modelica.Icons.Function;

              input Modelica.SIunits.MassFlowRate m_flow
                "Mass flow rate in design flow direction";
              input Real k(unit="")
                "Flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
              input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
                "Mass flow rate where transition to turbulent flow occurs";
              input Real m_flow_der(unit="kg/s2")
                "1st derivative of mass flow rate in design flow direction";
              input Real m_flow_der2(unit="kg/s3")
                "2nd derivative of mass flow rate in design flow direction";
              output Real dp_der2
                "2nd derivative of pressure difference between port_a and port_b (= port_a.p - port_b.p)";
            protected
              Modelica.SIunits.PressureDifference dp_turbulent = (m_flow_turbulent/k)^2
                "Pressure where flow changes to turbulent";
              Real m_flowNorm = m_flow/m_flow_turbulent
                "Normalised mass flow rate";
              Real m_flowNormSq = m_flowNorm^2
                "Square of normalised mass flow rate";
            algorithm
             dp_der2 :=if noEvent(abs(m_flow)>m_flow_turbulent)
                       then sign(m_flow)*2/k^2 * (m_flow_der^2 + m_flow * m_flow_der2)
                       else dp_turbulent/m_flow_turbulent*(
                             (0.375  + (2.25 - 0.625*m_flowNormSq)*m_flowNormSq)*m_flow_der2
                           + (4.5 - 2.5*m_flowNormSq)*m_flowNorm/m_flow_turbulent*m_flow_der^2);

             annotation (smoothOrder=0,
             Inline=false,
            Documentation(info="<html>
<p>
Function that implements the second order derivative of
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow</a>
with respect to the mass flow rate.
</p>
</html>",   revisions="<html>
<ul>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 29, 2015, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
            end basicFlowFunction_m_flow_der2;
          annotation (Documentation(info="<html>
<p>
This package contains a basic flow model that is used by the
various models that compute pressure drop.
</p>
<h4>Assumption and limitations</h4>
<p>
Because the density does not change signficantly in heating,
ventilation and air conditioning systems for buildings,
the flow models compute the pressure drop based on the mass flow
rate and not the volume flow rate. This typically leads to simpler
equations because it does not require
the mass density, which changes when the flow is reversed.
Although, for conceptual design of building energy system, there is
in general not enough information available that would warrant a more
detailed pressure drop calculation.
If a more detailed computation of the flow resistance is needed,
then a user can use models from the
<a href=\"modelica://Modelica.Fluid\">Modelica.Fluid</a> library.
</p>
<p>
All functions have an argument <code>m_flow_turbulent</code> that determines where the
flow transitions to fully turbulent flow. For smaller mass flow rates,
the quadratic relation is replaced by a function that has finite slope
near zero pressure drop. This is done for numerical reasons, and to approximate
laminar flow, although the implementation does not use a linear function.
</p>
<h4>Implementation</h4>
<p>
The two main functions are
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_dp</a>
and
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow</a>
that compute the mass flow rate or the pressure drop, respectively.
Both functions are two times continuously differentiable.
First and second order derivatives are provided
in the function that have the suffix <code>_der</code> and <code>_der2</code>.
</p>
</html>",           revisions="<html>
<ul>
<li>
April 10, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
          end FlowModels;
        annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid\">Buildings.Fluid</a>.
</p>
</html>"));
        end BaseClasses;
      annotation (
      preferredView="info", Documentation(info="<html>
This package contains components for fluid flow systems such as
pumps, valves and sensors. For other fluid flow models, see
<a href=\"modelica://Modelica.Fluid\">Modelica.Fluid</a>.
</html>"),
      Icon(graphics={
              Polygon(points={{-70,26},{68,-44},{68,26},{2,-10},{-70,-42},{-70,26}},
                  lineColor={0,0,0}),
              Line(points={{2,42},{2,-10}}, color={0,0,0}),
              Rectangle(
                extent={{-18,50},{22,42}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid)}));
      end Fluid;

      package HeatTransfer "Package with heat transfer models"
        extends Modelica.Icons.Package;

        package Sources "Thermal sources"
        extends Modelica.Icons.SourcesPackage;

          model PrescribedTemperature "Variable temperature boundary condition in Kelvin"

            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port annotation (Placement(transformation(extent={{90,
                      -10},{110,10}})));
            Modelica.Blocks.Interfaces.RealInput T annotation (Placement(transformation(
                    extent={{-140,-20},{-100,20}})));
          equation
            port.T = T;
            annotation (
              Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                      100,100}}), graphics={
                  Rectangle(
                    extent={{-100,100},{100,-100}},
                    lineColor={0,0,0},
                    pattern=LinePattern.None,
                    fillColor={159,159,223},
                    fillPattern=FillPattern.Backward),
                  Line(
                    points={{-102,0},{64,0}},
                    color={191,0,0},
                    thickness=0.5),
                  Text(
                    extent={{0,0},{-100,-100}},
                    lineColor={0,0,0},
                    textString="K"),
                  Text(
                    extent={{-150,150},{150,110}},
                    textString="%name",
                    lineColor={0,0,255}),
                  Polygon(
                    points={{50,-20},{50,20},{90,0},{50,-20}},
                    lineColor={191,0,0},
                    fillColor={191,0,0},
                    fillPattern=FillPattern.Solid)}),
              Documentation(info="<html>
<p>
This model represents a variable temperature boundary condition.
The temperature in [K] is given as input signal <b>T</b>
to the model. The effect is that an instance of this model acts as
an infinite reservoir able to absorb or generate as much energy
as required to keep the temperature at the specified value.
</p>
</html>"),           Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                      100,100}}), graphics={
                  Rectangle(
                    extent={{-100,100},{100,-100}},
                    lineColor={0,0,0},
                    pattern=LinePattern.None,
                    fillColor={159,159,223},
                    fillPattern=FillPattern.Backward),
                  Text(
                    extent={{0,0},{-100,-100}},
                    lineColor={0,0,0},
                    textString="K"),
                  Line(
                    points={{-102,0},{64,0}},
                    color={191,0,0},
                    thickness=0.5),
                  Polygon(
                    points={{52,-20},{52,20},{90,0},{52,-20}},
                    lineColor={191,0,0},
                    fillColor={191,0,0},
                    fillPattern=FillPattern.Solid)}));
          end PrescribedTemperature;
          annotation (   Documentation(info="<html>
This package is identical to
<a href=\"modelica://Modelica.Thermal.HeatTransfer.Sources\">
Modelica.Thermal.HeatTransfer.Sources</a>, except that
the parameters <code>alpha</code> and <code>T_ref</code> have
been deleted in the models
<a href=\"modelica://Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow\">
Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow</a> and
<a href=\"modelica://Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow\">
Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow</a>
 as these can cause division by zero in some fluid flow models.
</html>"));
        end Sources;
      annotation (preferredView="info", Documentation(info="<html>
This package contains models for heat transfer elements.
</html>"),
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}),
      graphics={
            Polygon(
              origin = {13.758,27.517},
              lineColor = {128,128,128},
              fillColor = {192,192,192},
              fillPattern = FillPattern.Solid,
              points = {{-54,-6},{-61,-7},{-75,-15},{-79,-24},{-80,-34},{-78,-42},{-73,-49},{-64,-51},{-57,-51},{-47,-50},{-41,-43},{-38,-35},{-40,-27},{-40,-20},{-42,-13},{-47,-7},{-54,-5},{-54,-6}}),
          Polygon(
              origin = {13.758,27.517},
              fillColor = {160,160,164},
              fillPattern = FillPattern.Solid,
              points = {{-75,-15},{-79,-25},{-80,-34},{-78,-42},{-72,-49},{-64,-51},{-57,-51},{-47,-50},{-57,-47},{-65,-45},{-71,-40},{-74,-33},{-76,-23},{-75,-15},{-75,-15}}),
            Polygon(
              origin = {13.758,27.517},
              lineColor = {160,160,164},
              fillColor = {192,192,192},
              fillPattern = FillPattern.Solid,
              points = {{39,-6},{32,-7},{18,-15},{14,-24},{13,-34},{15,-42},{20,-49},{29,-51},{36,-51},{46,-50},{52,-43},{55,-35},{53,-27},{53,-20},{51,-13},{46,-7},{39,-5},{39,-6}}),
            Polygon(
              origin = {13.758,27.517},
              fillColor = {160,160,164},
              fillPattern = FillPattern.Solid,
              points = {{18,-15},{14,-25},{13,-34},{15,-42},{21,-49},{29,-51},{36,-51},{46,-50},{36,-47},{28,-45},{22,-40},{19,-33},{17,-23},{18,-15},{18,-15}}),
            Polygon(
              origin = {13.758,27.517},
              lineColor = {191,0,0},
              fillColor = {191,0,0},
              fillPattern = FillPattern.Solid,
              points = {{-9,-23},{-9,-10},{18,-17},{-9,-23}}),
            Line(
              origin = {13.758,27.517},
              points = {{-41,-17},{-9,-17}},
              color = {191,0,0},
              thickness = 0.5),
            Line(
              origin = {13.758,27.517},
              points = {{-17,-40},{15,-40}},
              color = {191,0,0},
              thickness = 0.5),
            Polygon(
              origin = {13.758,27.517},
              lineColor = {191,0,0},
              fillColor = {191,0,0},
              fillPattern = FillPattern.Solid,
              points = {{-17,-46},{-17,-34},{-40,-40},{-17,-46}})}));
      end HeatTransfer;

      package Utilities "Package with utility functions such as for I/O"
        extends Modelica.Icons.Package;

        package Math "Library with functions such as for smoothing"
          extends Modelica.Icons.Package;

          package Functions "Package with mathematical functions"
            extends Modelica.Icons.VariantsPackage;

            function inverseXRegularized
              "Function that approximates 1/x by a twice continuously differentiable function"
              extends Modelica.Icons.Function;
             input Real x "Abscissa value";
             input Real delta(min=Modelica.Constants.eps)
                "Abscissa value below which approximation occurs";
             input Real deltaInv = 1/delta "Inverse value of delta";

             input Real a = -15*deltaInv "Polynomial coefficient";
             input Real b = 119*deltaInv^2 "Polynomial coefficient";
             input Real c = -361*deltaInv^3 "Polynomial coefficient";
             input Real d = 534*deltaInv^4 "Polynomial coefficient";
             input Real e = -380*deltaInv^5 "Polynomial coefficient";
             input Real f = 104*deltaInv^6 "Polynomial coefficient";

             output Real y "Function value";

            algorithm
              y := if (x > delta or x < -delta) then 1/x elseif (x < delta/2 and x > -delta/2) then x/(delta*delta) else Buildings.Utilities.Math.Functions.BaseClasses.smoothTransition(
                x=x,
                delta=delta,
                deltaInv=deltaInv,
                a=a,
                b=b,
                c=c,
                d=d,
                e=e,
                f=f);

              annotation (smoothOrder=2,
              derivative(order=1,
                      zeroDerivative=delta,
                      zeroDerivative=deltaInv,
                      zeroDerivative=a,
                      zeroDerivative=b,
                      zeroDerivative=c,
                      zeroDerivative=d,
                      zeroDerivative=e,
                      zeroDerivative=f)=Buildings.Utilities.Math.Functions.BaseClasses.der_inverseXRegularized,
                          Inline=true,
            Documentation(info="<html>
<p>
Function that approximates <i>y=1 &frasl; x</i>
inside the interval <i>-&delta; &le; x &le; &delta;</i>.
The approximation is twice continuously differentiable with a bounded derivative on the whole
real line.
</p>
<p>
See the plot of
<a href=\"modelica://Buildings.Utilities.Math.Functions.Examples.InverseXRegularized\">
Buildings.Utilities.Math.Functions.Examples.InverseXRegularized</a>
for the graph.
</p>
<p>
For efficiency, the polynomial coefficients
<code>a, b, c, d, e, f</code> and
the inverse of the smoothing parameter <code>deltaInv</code>
are exposed as arguments to this function.
Typically, these coefficients only depend on parameters and hence
can be computed once.
They must be equal to their default values, otherwise the function
is not twice continuously differentiable.
By exposing these coefficients as function arguments, models
that call this function can compute them as parameters, and
assign these parameter values in the function call.
This avoids that the coefficients are evaluated for each time step,
as they would otherwise be if they were to be computed inside the
body of the function. However, assigning the values is optional
as otherwise, at the expense of efficiency, the values will be
computed each time the function is invoked.
See
<a href=\"modelica://Buildings.Utilities.Math.Functions.Examples.InverseXRegularized\">
Buildings.Utilities.Math.Functions.Examples.InverseXRegularized</a>
for how to efficiently call this function.
</p>
</html>",             revisions="<html>
<ul>
<li>
August 10, 2015, by Michael Wetter:<br/>
Removed dublicate entry <code>smoothOrder = 1</code>
and reimplmented the function so it is twice continuously differentiable.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/302\">issue 302</a>.
</li>
<li>
February 5, 2015, by Filip Jorissen:<br/>
Added <code>smoothOrder = 1</code>.
</li>
<li>
May 10, 2013, by Michael Wetter:<br/>
Reformulated implementation to avoid unrequired computations.
</li>
<li>
April 18, 2011, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
            end inverseXRegularized;

            function regStep
              "Approximation of a general step, such that the approximation is continuous and differentiable"
              extends Modelica.Icons.Function;
              input Real x "Abscissa value";
              input Real y1 "Ordinate value for x > 0";
              input Real y2 "Ordinate value for x < 0";
              input Real x_small(min=0) = 1e-5
                "Approximation of step for -x_small <= x <= x_small; x_small >= 0 required";
              output Real y "Ordinate value to approximate y = if x > 0 then y1 else y2";
            algorithm
              y := smooth(1, if x >  x_small then y1 else
                             if x < -x_small then y2 else
                             if x_small > 0 then (x/x_small)*((x/x_small)^2 - 3)*(y2-y1)/4 + (y1+y2)/2 else (y1+y2)/2);

              annotation(Inline=true,
              Documentation(revisions="<html>
<ul>
<li><i>February 18, 2016</i>
    by Marcus Fuchs:<br/>
    Add function with <code>Inline = true</code> in annotations to package for better performance,
    as suggested in <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/300\">#300</a> .</li>
<li><i>April 29, 2008</i>
    by <a href=\"mailto:Martin.Otter@DLR.de\">Martin Otter</a>:<br/>
    Designed and implemented.</li>
<li><i>August 12, 2008</i>
    by <a href=\"mailto:Michael.Sielemann@dlr.de\">Michael Sielemann</a>:<br/>
    Minor modification to cover the limit case <code>x_small -> 0</code> without division by zero.</li>
</ul>
</html>",             info="<html>
<p>
This function is used to approximate the equation
</p>
<pre>
    y = <b>if</b> x &gt; 0 <b>then</b> y1 <b>else</b> y2;
</pre>

<p>
by a smooth characteristic, so that the expression is continuous and differentiable:
</p>

<pre>
   y = <b>smooth</b>(1, <b>if</b> x &gt;  x_small <b>then</b> y1 <b>else</b>
                 <b>if</b> x &lt; -x_small <b>then</b> y2 <b>else</b> f(y1, y2));
</pre>

<p>
In the region <code>-x_small &lt; x &lt; x_small</code> a 2nd order polynomial is used
for a smooth transition from <code>y1</code> to <code>y2</code>.
</p>
</html>"));
            end regStep;

            function spliceFunction
              extends Modelica.Icons.Function;
                input Real pos "Argument of x > 0";
                input Real neg "Argument of x < 0";
                input Real x "Independent value";
                input Real deltax "Half width of transition interval";
                output Real out "Smoothed value";
            protected
                Real scaledX1;
                Real y;
                constant Real asin1 = Modelica.Math.asin(1);
            algorithm
                scaledX1 := x/deltax;
                if scaledX1 <= -0.999999999 then
                  out := neg;
                elseif scaledX1 >= 0.999999999 then
                  out := pos;
                else
                  y := (Modelica.Math.tanh(Modelica.Math.tan(scaledX1*asin1)) + 1)/2;
                  out := pos*y + (1 - y)*neg;
                end if;

                annotation (
            smoothOrder=1,
            derivative=BaseClasses.der_spliceFunction,
            Documentation(info="<html>
<p>
Function to provide a once continuously differentiable transition between
to arguments.
</p><p>
The function is adapted from
<a href=\"modelica://Modelica.Media.Air.MoistAir.Utilities.spliceFunction\">
Modelica.Media.Air.MoistAir.Utilities.spliceFunction</a> and provided here
for easier accessability to model developers.
</p>
</html>",             revisions="<html>
<ul>
<li>
May 10, 2013, by Michael Wetter:<br/>
Reformulated implementation to avoid unrequired computations.
</li>
<li>
May 11, 2010, by Michael Wetter:<br/>
Removed default value for transition interval as this is problem dependent.
</li>
<li>
May 20, 2008, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
            end spliceFunction;

            package BaseClasses "Package with base classes for Buildings.Utilities.Math.Functions"
              extends Modelica.Icons.BasesPackage;

              function der_2_smoothTransition
                "Second order derivative of smoothTransition with respect to x"
                extends Modelica.Icons.Function;
                input Real x "Abscissa value";
                input Real delta(min=Modelica.Constants.eps)
                  "Abscissa value below which approximation occurs";

                input Real deltaInv "Inverse value of delta";
                input Real a "Polynomial coefficient";
                input Real b "Polynomial coefficient";
                input Real c "Polynomial coefficient";
                input Real d "Polynomial coefficient";
                input Real e "Polynomial coefficient";
                input Real f "Polynomial coefficient";

                input Real x_der "Derivative of x";
                input Real x_der2 "Second order derivative of x";
                output Real y_der2 "Second order derivative of function value";
              protected
                Real aX "Absolute value of x";
                Real ex "Intermediate expression";
              algorithm
               aX:= abs(x);
               ex     := 2*c + aX*(6*d + aX*(12*e + aX*20*f));
               y_der2 := (b + aX*(2*c + aX*(3*d + aX*(4*e + aX*5*f))))*x_der2
                       + x_der*x_der*( if x > 0 then ex else -ex);

              annotation (
              Documentation(info="<html>
<p>
This function is the 2nd order derivative of
<a href=\"modelica://Buildings.Utilities.Math.Functions.BaseClasses.smoothTransition\">
Buildings.Utilities.Math.Functions.BaseClasses.smoothTransition</a>.
</p>
<h4>Implementation</h4>
<p>
For efficiency, the polynomial coefficients
<code>a, b, c, d, e, f</code> and
the inverse of the smoothing parameter <code>deltaInv</code>
are exposed as arguments to this function.
</p>
</html>",               revisions="<html>
<ul>
<li>
August 11, 2015, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
              end der_2_smoothTransition;

              function der_inverseXRegularized "Derivative of inverseXRegularised function"
                extends Modelica.Icons.Function;
               input Real x "Abscissa value";
               input Real delta(min=Modelica.Constants.eps)
                  "Abscissa value below which approximation occurs";
               input Real deltaInv = 1/delta "Inverse value of delta";

               input Real a = -15*deltaInv "Polynomial coefficient";
               input Real b = 119*deltaInv^2 "Polynomial coefficient";
               input Real c = -361*deltaInv^3 "Polynomial coefficient";
               input Real d = 534*deltaInv^4 "Polynomial coefficient";
               input Real e = -380*deltaInv^5 "Polynomial coefficient";
               input Real f = 104*deltaInv^6 "Polynomial coefficient";

               input Real x_der "Abscissa value";
               output Real y_der "Function value";

              algorithm
                y_der := if (x > delta or x < -delta) then -x_der/x/x elseif (x < delta/2 and x > -delta/2) then x_der/(delta*delta) else Buildings.Utilities.Math.Functions.BaseClasses.der_smoothTransition(
                  x=x,
                  x_der=x_der,
                  delta=delta,
                  deltaInv=deltaInv,
                  a=a,
                  b=b,
                  c=c,
                  d=d,
                  e=e,
                  f=f);
              annotation (
              Documentation(
              info="<html>
<p>
Implementation of the first derivative of the function
<a href=\"modelica://Buildings.Utilities.Math.Functions.inverseXRegularized\">
Buildings.Utilities.Math.Functions.inverseXRegularized</a>.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 22, 2016, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>"));
              end der_inverseXRegularized;

              function der_smoothTransition
                "First order derivative of smoothTransition with respect to x"
                extends Modelica.Icons.Function;
                input Real x "Abscissa value";
                input Real delta(min=Modelica.Constants.eps)
                  "Abscissa value below which approximation occurs";

                input Real deltaInv "Inverse value of delta";
                input Real a "Polynomial coefficient";
                input Real b "Polynomial coefficient";
                input Real c "Polynomial coefficient";
                input Real d "Polynomial coefficient";
                input Real e "Polynomial coefficient";
                input Real f "Polynomial coefficient";

                input Real x_der "Derivative of x";
                output Real y_der "Derivative of function value";

              protected
                Real aX "Absolute value of x";
              algorithm
               aX:= abs(x);
               y_der := (b + aX*(2*c + aX*(3*d + aX*(4*e + aX*5*f))))*x_der;
               annotation(smoothOrder=1,
                        derivative(order=2,
                        zeroDerivative=delta,
                        zeroDerivative=deltaInv,
                        zeroDerivative=a,
                        zeroDerivative=b,
                        zeroDerivative=c,
                        zeroDerivative=d,
                        zeroDerivative=e,
                        zeroDerivative=f)=Buildings.Utilities.Math.Functions.BaseClasses.der_2_smoothTransition,
              Documentation(info="<html>
<p>
This function is the 1st order derivative of
<a href=\"modelica://Buildings.Utilities.Math.Functions.BaseClasses.smoothTransition\">
Buildings.Utilities.Math.Functions.BaseClasses.smoothTransition</a>.
</p>
<h4>Implementation</h4>
<p>
For efficiency, the polynomial coefficients
<code>a, b, c, d, e, f</code> and
the inverse of the smoothing parameter <code>deltaInv</code>
are exposed as arguments to this function.
Also,
its derivative is provided in
<a href=\"modelica://Buildings.Utilities.Math.Functions.BaseClasses.der_2_smoothTransition\">
Buildings.Utilities.Math.Functions.BaseClasses.der_2__smoothTransition</a>.
</p>
</html>",               revisions="<html>
<ul>
<li>
August 11, 2015, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
              end der_smoothTransition;

              function der_spliceFunction "Derivative of splice function"
                extends Modelica.Icons.Function;
                  input Real pos;
                  input Real neg;
                  input Real x;
                  input Real deltax=1;
                  input Real dpos;
                  input Real dneg;
                  input Real dx;
                  input Real ddeltax=0;
                  output Real out;
              protected
                  Real scaledX;
                  Real scaledX1;
                  Real dscaledX1;
                  Real y;
                  constant Real asin1 = Modelica.Math.asin(1);
              algorithm
                  scaledX1 := x/deltax;
                  if scaledX1 <= -0.99999999999 then
                    out := dneg;
                  elseif scaledX1 >= 0.9999999999 then
                    out := dpos;
                  else
                    scaledX := scaledX1*asin1;
                    dscaledX1 := (dx - scaledX1*ddeltax)/deltax;
                    y := (Modelica.Math.tanh(Modelica.Math.tan(scaledX)) + 1)/2;
                    out := dpos*y + (1 - y)*dneg;
                    out := out + (pos - neg)*dscaledX1*asin1/2/(
                      Modelica.Math.cosh(Modelica.Math.tan(scaledX))*Modelica.Math.cos(
                      scaledX))^2;
                  end if;

              annotation (
              Documentation(
              info="<html>
<p>
Implementation of the first derivative of the function
<a href=\"modelica://Buildings.Utilities.Math.Functions.spliceFunction\">
Buildings.Utilities.Math.Functions.spliceFunction</a>.
</p>
</html>",     revisions="<html>
<ul>
<li>
May 10, 2013, by Michael Wetter:<br/>
Reformulated implementation to avoid unrequired computations.
</li>
<li>
April 7, 2009, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
              end der_spliceFunction;

              function smoothTransition
                "Twice continuously differentiable transition between the regions"
                extends Modelica.Icons.Function;

                // The function that transitions between the regions is implemented
                // using its own function. This allows Dymola 2016 to inline the function
                // inverseXRegularized.

              input Real x "Abscissa value";
              input Real delta(min=Modelica.Constants.eps)
                  "Abscissa value below which approximation occurs";
              input Real deltaInv = 1/delta "Inverse value of delta";

              input Real a = -15*deltaInv "Polynomial coefficient";
              input Real b = 119*deltaInv^2 "Polynomial coefficient";
              input Real c = -361*deltaInv^3 "Polynomial coefficient";
              input Real d = 534*deltaInv^4 "Polynomial coefficient";
              input Real e = -380*deltaInv^5 "Polynomial coefficient";
              input Real f = 104*deltaInv^6 "Polynomial coefficient";
              output Real y "Function value";
              protected
                Real aX "Absolute value of x";

              algorithm
               aX:= abs(x);
               y := (if x >= 0 then 1 else -1) * (a + aX*(b + aX*(c + aX*(d + aX*(e + aX*f)))));
              annotation(smoothOrder=2,
                derivative(order=1,
                        zeroDerivative=delta,
                        zeroDerivative=deltaInv,
                        zeroDerivative=a,
                        zeroDerivative=b,
                        zeroDerivative=c,
                        zeroDerivative=d,
                        zeroDerivative=e,
                        zeroDerivative=f)=Buildings.Utilities.Math.Functions.BaseClasses.der_smoothTransition,
                  Documentation(info="<html>
<p>
This function is used by
<a href=\"modelica://Buildings.Utilities.Math.Functions.inverseXRegularized\">
Buildings.Utilities.Math.Functions.inverseXRegularized</a>
to provide a twice continuously differentiable transition between
the different regions.
The code has been implemented in a function as this allows
to implement the function
<a href=\"modelica://Buildings.Utilities.Math.Functions.inverseXRegularized\">
Buildings.Utilities.Math.Functions.inverseXRegularized</a>
in such a way that Dymola inlines it.
However, this function will not be inlined as its body is too large.
</p>
<h4>Implementation</h4>
<p>
For efficiency, the polynomial coefficients
<code>a, b, c, d, e, f</code> and
the inverse of the smoothing parameter <code>deltaInv</code>
are exposed as arguments to this function.
Also,
derivatives are provided in
<a href=\"modelica://Buildings.Utilities.Math.Functions.BaseClasses.der_smoothTransition\">
Buildings.Utilities.Math.Functions.BaseClasses.der_smoothTransition</a>
and in
<a href=\"modelica://Buildings.Utilities.Math.Functions.BaseClasses.der_2_smoothTransition\">
Buildings.Utilities.Math.Functions.BaseClasses.der_2__smoothTransition</a>.
</p>
</html>",               revisions="<html>
<ul>
<li>
September 12, 2018, by David Blum:<br/>
Change if-statement to if-expression.  
For issue <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1019\">#1019</a>.
</li>
<li>
August 11, 2015, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
              end smoothTransition;
            annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Utilities.Math.Functions\">Buildings.Utilities.Math.Functions</a>.
</p>
</html>"));
            end BaseClasses;
          annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains functions for commonly used
mathematical operations. The functions are used in
the blocks
<a href=\"modelica://Buildings.Utilities.Math\">
Buildings.Utilities.Math</a>.
</p>
</html>"));
          end Functions;
        annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains blocks and functions for commonly used
mathematical operations.
The classes in this package augment the classes
<a href=\"modelica://Modelica.Blocks\">
Modelica.Blocks</a>.
</p>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Line(points={{-80,0},{-68.7,34.2},{-61.5,53.1},
                    {-55.1,66.4},{-49.4,74.6},{-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{
                    -26.9,69.7},{-21.3,59.4},{-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,
                    -50.2},{23.7,-64.2},{29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},
                    {51.9,-71.5},{57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}}, color={
                    0,0,0}, smooth=Smooth.Bezier)}));
        end Math;
      annotation (
      preferredView="info", Documentation(info="<html>
<p>
This package contains utility models such as for thermal comfort calculation, input/output, co-simulation, psychrometric calculations and various functions that are used throughout the library.
</p>
</html>"),
      Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
          Polygon(
            origin={1.3835,-4.1418},
            rotation=45.0,
            fillColor={64,64,64},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-15.0,93.333},{-15.0,68.333},{0.0,58.333},{15.0,68.333},{15.0,93.333},{20.0,93.333},{25.0,83.333},{25.0,58.333},{10.0,43.333},{10.0,-41.667},{25.0,-56.667},{25.0,-76.667},{10.0,-91.667},{0.0,-91.667},{0.0,-81.667},{5.0,-81.667},{15.0,-71.667},{15.0,-61.667},{5.0,-51.667},{-5.0,-51.667},{-15.0,-61.667},{-15.0,-71.667},{-5.0,-81.667},{0.0,-81.667},{0.0,-91.667},{-10.0,-91.667},{-25.0,-76.667},{-25.0,-56.667},{-10.0,-41.667},{-10.0,43.333},{-25.0,58.333},{-25.0,83.333},{-20.0,93.333}}),
          Polygon(
            origin={10.1018,5.218},
            rotation=-45.0,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            points={{-15.0,87.273},{15.0,87.273},{20.0,82.273},{20.0,27.273},{10.0,17.273},{10.0,7.273},{20.0,2.273},{20.0,-2.727},{5.0,-2.727},{5.0,-77.727},{10.0,-87.727},{5.0,-112.727},{-5.0,-112.727},{-10.0,-87.727},{-5.0,-77.727},{-5.0,-2.727},{-20.0,-2.727},{-20.0,2.273},{-10.0,7.273},{-10.0,17.273},{-20.0,27.273},{-20.0,82.273}})}));
      end Utilities;
    annotation (
    preferredView="info",
    versionDate="2020-05-28",
    dateModified="2020-05-28",
    conversion(
      from(version={"6.0.0"},
          script="modelica://Buildings/Resources/Scripts/Dymola/ConvertBuildings_from_6_to_7.0.0.mos")),
    preferredView="info",
    Documentation(info="<html>
<p>
The <code>Buildings</code> library is a free library
for modeling building energy and control systems.
Many models are based on models from the package
<code>Modelica.Fluid</code> and use
the same ports to ensure compatibility with the Modelica Standard
Library.
</p>
<p>
The figure below shows a section of the schematic view of the model
<a href=\"modelica://Buildings.Examples.HydronicHeating\">
Buildings.Examples.HydronicHeating</a>.
In the lower part of the figure, there is a dynamic model of a boiler, a pump and a stratified energy storage tank. Based on the temperatures of the storage tank, a finite state machine switches the boiler and its pump on and off.
The heat distribution is done using a hydronic heating system with a three way valve and a pump with variable revolutions. The upper right hand corner shows a room model that is connected to a radiator whose flow is controlled by a thermostatic valve.
</p>
<p align=\"center\">
<img alt=\"image\" src=\"modelica://Buildings/Resources/Images/UsersGuide/HydronicHeating.png\" border=\"1\"/>
</p>
<p>
The web page for this library is
<a href=\"http://simulationresearch.lbl.gov/modelica\">http://simulationresearch.lbl.gov/modelica</a>,
and the development page is
<a href=\"https://github.com/lbl-srg/modelica-buildings\">https://github.com/lbl-srg/modelica-buildings</a>.
Contributions to further advance the library are welcomed.
Contributions may not only be in the form of model development, but also
through model use, model testing,
requirements definition or providing feedback regarding the model applicability
to solve specific problems.
</p>
</html>"));
    end Buildings;
  end Core;

  package Components

    package Network "Components for district heating network"

      model IsoplusPipe
        "Single Pipe defined by Isoplus catalogue. Derived from DisHeatLib DualPipe"

        replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater constrainedby Modelica.Media.Interfaces.PartialMedium "Medium in the component"
          annotation (choicesAllMatching=true);

        extends Core.Buildings.Fluid.Interfaces.PartialTwoPortInterface;
        //extends DisHeatLib.BaseClasses.PartialFourPortVectorInterface(
        //  redeclare package Medium1 = Medium,
        //  redeclare paackage Medium2 = Medium,
        //  final m1_flow_nominal = pipeType.m_flow_nominal,
        //  final m2_flow_nominal = pipeType.m_flow_nominal,
        //  final m1_flow_small = m_flow_small,
        //  final m2_flow_small = m_flow_small,
        //  final allowFlowReversal1 = allowFlowReversal,
        //  final allowFlowReversal2 = allowFlowReversal);


        replaceable parameter BaseClasses.BasePipe pipeType "Pipe type"
        annotation (choicesAllMatching=true);

        parameter Modelica.SIunits.MassFlowRate m_flow_nominal=pipeType.m_flow_nominal
          "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

        parameter Boolean allowFlowReversal = true
          "= false to simplify equations, assuming, but not enforcing, no flow reversal"
          annotation(Dialog(tab="Assumptions"), Evaluate=true);
        parameter Medium.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
          "Small mass flow rate for regularization of zero flow"
          annotation(Dialog(tab = "Advanced"));
        parameter Boolean from_dp = false
          "= true, use m_flow = f(dp) else dp = f(m_flow)"
          annotation (Evaluate=true, Dialog(tab="Flow resistance"));
        parameter Boolean linearizeFlowResistance = false
          "= true, use linear relation between m_flow and dp for any flow rate"
          annotation(Dialog(tab="Flow resistance"));
        // Parameters
        parameter Modelica.SIunits.Length L "Length of the pipes";
        parameter Real ReC=4000
          "Reynolds number where transition to turbulent starts"
          annotation (Evaluate=true, Dialog(group="Additional parameters"));
        parameter Real fac=1
          "Factor to take into account flow resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal"
          annotation (Evaluate=true, Dialog(group="Additional parameters"));
        parameter Real cf = 1.0 "Correction factor of heat losses (needed for aggregation)"
          annotation (Evaluate=true, Dialog(group="Additional parameters"));

        // Initialization
        parameter Modelica.SIunits.Temperature T_init = Medium.T_default "Initial temperature of supply pipe"
          annotation(Dialog(tab = "Initialization"));

        Modelica.SIunits.Power Q_flow "Heat flow to soil (i.e., losses)";
          Modelica.SIunits.ThermodynamicTemperature T_a "Temperature at port_a";
        Modelica.SIunits.ThermodynamicTemperature T_b "Temperature at port_b";
          Modelica.SIunits.HeatFlowRate Pth_grid  "Heat flow, Pth_grid > 0: fed into grid, Pth_grid < 0: taken from grid";

      protected
        Core.Buildings.Fluid.FixedResistances.PlugFlowPipe pipe_sl(
          redeclare package Medium = Medium,
          from_dp=from_dp,
          ReC=ReC,
          roughness=pipeType.pipeMaterial.roughness,
          m_flow_small=m_flow_small,
          dIns=pipeType.dIns,
          length=L,
          m_flow_nominal=pipeType.m_flow_nominal,
          T_start_in=T_init,
          T_start_out=T_init,
          cPip=pipeType.pipeMaterial.cPip,
          rhoPip=pipeType.pipeMaterial.rhoPip,
          v_nominal=pipeType.v_nominal,
          dh=pipeType.dh,
          thickness=pipeType.dWall,
          initDelay=true,
          m_flow_start=m_flow_nominal,
          fac=fac,
          linearized=linearizeFlowResistance,
          allowFlowReversal=allowFlowReversal,
          kIns=cf*pipeType.kIns,
          nPorts=1)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      public
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
          annotation (Placement(transformation(extent={{-10,90},{10,110}})));
      equation
        Q_flow = heatPort.Q_flow;

         Pth_grid = if port_a.m_flow > 0 then -abs(port_a.m_flow*(port_b.h_outflow - inStream(port_a.h_outflow))) else -abs(port_a.m_flow*(inStream(port_b.h_outflow) - port_a.h_outflow));
          T_a = if port_a.m_flow < 0 then Medium.temperature_phX(port_a.p, port_a.h_outflow, X = port_a.Xi_outflow) else Medium.temperature_phX(port_a.p, inStream(port_a.h_outflow), X = port_a.Xi_outflow);
          T_b = if port_a.m_flow < 0 then Medium.temperature_phX(port_b.p, inStream(port_b.h_outflow), X = port_b.Xi_outflow) else Medium.temperature_phX(port_b.p, port_b.h_outflow, X = port_b.Xi_outflow);

        connect(pipe_sl.heatPort, heatPort)
          annotation (Line(points={{0,10},{0,100}},        color={191,0,0}));
        connect(port_a, pipe_sl.port_a)
          annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
        connect(pipe_sl.ports_b[1], port_b)
          annotation (Line(points={{10,0},{100,0}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,40},{100,-40}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={255,0,0}),
              Rectangle(
                extent={{-50,5},{50,-5}},
                lineColor=DynamicSelect({255,255,255}, if port_a.m_flow > 0.001 then {0,0,0} elseif port_a.m_flow < -0.001 then {255,255,255} else {0,220,0}),
                fillPattern=FillPattern.Solid,
                fillColor=DynamicSelect({255,255,255}, if port_a.m_flow > 0.001 then {0,0,0} elseif port_a.m_flow < -0.001 then {255,255,255} else {0,220,0})),
              Polygon(
                points=DynamicSelect({{40,20},{40,-20},{75,0}}, if port_a.m_flow > 0.001 then {{40,20},{40,-20},{75,0}} elseif port_a.m_flow < -0.001 then {{0,0},{0,0},{0,0}} else {{40,20},{40,-20},{75,0}}),
                lineColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {0,0,0}),
                lineThickness=0,
                fillPattern=FillPattern.Solid,
                fillColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {0,0,0})),
              Polygon(
                points=DynamicSelect({{-40,20},{-40,-20},{-75,0}}, if port_a.m_flow > 0.001 then {{0,0},{0,0},{0,0}} elseif port_a.m_flow < -0.001 then {{-40,20},{-40,-20},{-75,0}} else {{-40,20},{-40,-20},{-75,0}}),
                lineColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {255,255,255}),
                lineThickness=0,
                fillPattern=FillPattern.Solid,
                fillColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {255,255,255}))}),
          Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
      end IsoplusPipe;

      model Node
        "Simple node. Ideal mixing, no volume, no dynamic energy balance. Inspired by presentation of Modelica society 'Overview and Rationale for Modelica Stream Connectors'"

              replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater constrainedby Modelica.Media.Interfaces.PartialMedium "Medium in the component" annotation (choicesAllMatching=true);

              //Definitions for regularization (needed by ports)
              parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
               "Nominal mass flow rate"
                annotation(Dialog(group = "Nominal condition"));
              parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
              "Small mass flow rate for regularization of zero flow"
                annotation(Dialog(tab = "Advanced"));
              parameter Integer nPorts=0 "Number of ports"
               annotation(Dialog(connectorSizing=true)); //Automatic generation of Ports in Gui
              Modelica.Fluid.Interfaces.FluidPorts_b ports[nPorts](
                  redeclare each package Medium = Medium) "Fluid inlets and outlets"
                annotation (Placement(transformation(extent={{-15,-60},{15,60}},
                  origin={0,-101},
              rotation=90), iconTransformation(
              extent={{-15,-60},{15,60}},
              rotation=90,
              origin={0,-101})));

              // Modelica.SIunits.Pressure p = ports[1].p
                //  "Pressure of the fluid";
               //Medium.Temperature T = Medium.temperature_phX(p=p, h=medium.h, X=cat(1,Xi,{1-sum(Xi)}))
                 // "Temperature of the fluid";

                //Modelica.SIunits.MassFraction Xi[Medium.nXi] = XiOut_internal
                   // "Species concentration of the fluid";
                //Modelica.Blocks.Interfaces.RealOutput hOut_internal(unit="J/kg")
                    //"Internal connector for leaving temperature of the component";
              //Medium.ThermodynamicState state;

      //protected
        Medium.BaseProperties medium "Medium properties in node (e.g. port_a)";

        // Gridstate variables

         Modelica.SIunits.Pressure p = medium.p;
         Modelica.SIunits.Temperature T = medium.T;

      equation

        for i in 1:nPorts loop
          ports[i].p = medium.p;
          ports[i].h_outflow = medium.h;
        end for;
        //mass conservation
        0 = sum(ports[i].m_flow for i in 1:nPorts);
        //energy conservation
        0 = sum(ports[i].m_flow*actualStream(ports[i].h_outflow) for i in 1:nPorts);

                annotation(Evaluate=true, Dialog(connectorSizing=true, tab="General",group="Ports"),
                    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics={
             Text(
                extent={{-60,-26},{56,-58}},
                lineColor={255,255,255},
                textString="V=%V"),
              Text(
                extent={{-152,100},{148,140}},
                textString="%name",
                lineColor={0,0,255}),
             Ellipse(
                extent={{-100,98},{100,-102}},
                lineColor={0,0,0},
                fillPattern=FillPattern.Sphere,
                fillColor=DynamicSelect({170,213,255}, min(1, max(0, (1-(T-273.15)/50)))*{28,108,200}+min(1, max(0, (T-273.15)/50))*{255,0,0})),
              Text(
                extent={{62,28},{-58,-22}},
                lineColor={255,255,255},
                textString=DynamicSelect("", String(T-273.15, format=".1f")))}));
      end Node;

      package BaseClasses "Further definietions as material attributes, pipe types etc"
      extends Modelica.Icons.BasesPackage;

        record BasePipe "Record that contains the properties of a generic pipes"
          parameter Modelica.SIunits.Length dh=sqrt(4*m_flow_nominal/995/v_nominal/3.14) "Hydraulic diameter of pipe"
          annotation(Dialog(group="Nominal values"));
          parameter Modelica.SIunits.Velocity v_nominal=m_flow_nominal/(995*3.14*dh.^2/4) "Nominal velocity of flow"
          annotation(Dialog(group="Nominal values"));
          parameter Modelica.SIunits.MassFlowRate m_flow_nominal=995*v_nominal*3.14*dh.^2/4 "Nominal mass flow"
          annotation(Dialog(group="Nominal values"));
          replaceable parameter
            MeFlexWaermeLib.Components.Network.BaseClasses.BasePipeMaterial pipeMaterial
            "Material of pipe wall"
            annotation (Dialog(group="Pipe material"), choicesAllMatching=true);
          parameter Modelica.SIunits.Length dWall=0.0035 "Tickness of pipe wall"
          annotation(Dialog(group="Pipe material"));
          parameter Modelica.SIunits.ThermalConductivity kIns=0.024
            "Thermal conductivity of the insulation material"
            annotation(Dialog(group="Thermal insulation"));
          parameter Modelica.SIunits.Length dIns=0.1 "Tickness of insulation"
            annotation(Dialog(group="Thermal insulation"));
          annotation (Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>",         info="<html>
<p>This record contains all the properties that are specified for a district heating pipe. </p>
</html>"));
        end BasePipe;

        record BasePipeMaterial
          "Record that contains the properties of a pipe material"
          parameter Modelica.SIunits.Height roughness
            "Average height of surface asperities";
          parameter Modelica.SIunits.SpecificHeatCapacity cPip
            "Specific heat of pipe wall material";
          parameter Modelica.SIunits.Density rhoPip(displayUnit="kg/m3")
            "Density of pipe wall material";
          annotation (Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>",         info="<html>
<p>This record contains all the properties needed to specify the material of district heating pipe. </p>
</html>"));
        end BasePipeMaterial;

        package PipeMaterials "Records for different pipe materials"

          record Steel
            extends MeFlexWaermeLib.Components.Network.BaseClasses.BasePipeMaterial(
              roughness=2.5e-5,
              cPip=500,
              rhoPip=8000);
          annotation (Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
          end Steel;
        annotation (Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
        end PipeMaterials;

        package Isoplus
          extends Modelica.Icons.MaterialPropertiesPackage;

          record Isoplus_Std_DN200
            extends BasePipe(
              dh=0.2101,
              redeclare PipeMaterials.Steel pipeMaterial,
              dWall=0.0045,
              kIns=0.0270,
              dIns=0.04795,
              m_flow_nominal = 52.78);
            annotation (Documentation(revisions="<html>
<ul>
<li>November 02, 2020, by Pascal Friedrich:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
          end Isoplus_Std_DN200;

          record Isoplus_Std_DN300
            extends BasePipe(
              dh=0.3127,
              redeclare PipeMaterials.Steel pipeMaterial,
              dWall=0.0056,
              kIns=0.0270,
              dIns=0.06305,
              m_flow_nominal = 138.9);
            annotation (Documentation(revisions="<html>
<ul>
<li>November 02, 2020, by Pascal Friedrich:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
          end Isoplus_Std_DN300;
        annotation (Documentation(info="<html>
<p>These data records are based on the isoplus <a href=\"http://www.isoplus.at/produkte/starre-verbundsysteme/einzelrohr-konti.html\">Kontirohr-Einzel</a> spreadsheets with a <b>standard, single improved </b>and<b> double improved</b> insulation thickness. The thickness of insulation varies with the pipe diameter. The thermal conductivity, however, stays constant as the same insulation material (PUR-Hartschaumd&auml;mmung) is used.</p>
</html>",       revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
        end Isoplus;

        model PlugFlowCoreNoDelay "Pipe model using spatialDistribution for temperature delay without delay"
          extends Core.Buildings.Fluid.Interfaces.PartialTwoPort;

          constant Boolean homotopyInitialization = true "= true, use homotopy method"
            annotation(HideResult=true);

          parameter Modelica.SIunits.Length dh
            "Hydraulic diameter (assuming a round cross section area)";

          parameter Modelica.SIunits.Velocity v_nominal
            "Velocity at m_flow_nominal (used to compute default value for hydraulic diameter dh)"
            annotation(Dialog(group="Nominal condition"));

          parameter Modelica.SIunits.Length length(min=0) "Pipe length";

          parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
            "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

          parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(
            m_flow_nominal) "Small mass flow rate for regularization of zero flow"
            annotation (Dialog(tab="Advanced"));

          parameter Modelica.SIunits.Height roughness=2.5e-5
            "Average height of surface asperities (default: smooth steel pipe)"
            annotation (Dialog(group="Geometry"));

          parameter Real R(unit="(m.K)/W")
            "Thermal resistance per unit length from fluid to boundary temperature";

          parameter Real C(unit="J/(K.m)")
            "Thermal capacity per unit length of pipe";

          parameter Real fac=1
            "Factor to take into account flow resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal";

          parameter Boolean from_dp=false
            "= true, use m_flow = f(dp) else dp = f(m_flow)"
            annotation (Evaluate=true, Dialog(tab="Advanced"));
          parameter Modelica.SIunits.Length thickness(min=0) "Pipe wall thickness";

          parameter Modelica.SIunits.Temperature T_start_in=Medium.T_default
            "Initialization temperature at pipe inlet"
            annotation (Dialog(tab="Initialization"));
          parameter Modelica.SIunits.Temperature T_start_out=Medium.T_default
            "Initialization temperature at pipe outlet"
            annotation (Dialog(tab="Initialization"));
          parameter Boolean initDelay=true
            "Initialize delay for a constant mass flow rate if true, otherwise start from 0"
            annotation (Dialog(tab="Initialization"));
          parameter Modelica.SIunits.MassFlowRate m_flow_start=m_flow_nominal(min=0)
            annotation (Dialog(tab="Initialization", enable=initDelay));

          parameter Real ReC=4000
            "Reynolds number where transition to turbulent starts";

          parameter Boolean linearized = false
            "= true, use linear relation between m_flow and dp for any flow rate"
            annotation(Evaluate=true, Dialog(tab="Advanced"));

          Core.Buildings.Fluid.FixedResistances.HydraulicDiameter res(
            redeclare final package Medium = Medium,
            final dh=dh,
            final m_flow_nominal=m_flow_nominal,
            final from_dp=from_dp,
            final length=length,
            final roughness=roughness,
            final fac=fac,
            final ReC=ReC,
            final v_nominal=v_nominal,
            final allowFlowReversal=allowFlowReversal,
            final show_T=false,
            final homotopyInitialization=homotopyInitialization,
            final linearized=linearized,
            dp(nominal=fac*200*length)) "Pressure drop calculation for this pipe" annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

          Core.Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss heaLos_a(
            redeclare final package Medium = Medium,
            final C=C,
            final R=R,
            final m_flow_small=m_flow_small,
            final T_start=T_start_in,
            final m_flow_nominal=m_flow_nominal,
            final m_flow_start=m_flow_start,
            final show_T=false,
            final show_V_flow=false) "Heat loss for flow from port_b to port_a" annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));

          Core.Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss heaLos_b(
            redeclare final package Medium = Medium,
            final C=C,
            final R=R,
            final m_flow_small=m_flow_small,
            final T_start=T_start_out,
            final m_flow_nominal=m_flow_nominal,
            final m_flow_start=m_flow_start,
            final show_T=false,
            final show_V_flow=false) "Heat loss for flow from port_a to port_b" annotation (Placement(transformation(extent={{60,-10},{80,10}})));
          Core.Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare final package Medium = Medium) "Mass flow sensor" annotation (Placement(transformation(extent={{-50,10},{-30,-10}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
            "Heat port to connect environment (positive heat flow for heat loss to surroundings)"
            annotation (Placement(transformation(extent={{-10,90},{10,110}})));

          Modelica.Blocks.Math.Min min1 annotation (Placement(transformation(extent={{-46,-90},{-66,-110}})));
          Modelica.Blocks.Math.Max tau annotation (Placement(transformation(extent={{16,-100},{36,-120}})));
          Modelica.Blocks.Sources.RealExpression zeroConst annotation (Placement(transformation(extent={{-74,-138},{-54,-118}})));
          Modelica.Blocks.Math.Gain tauRev(k=-1) annotation (Placement(transformation(
                extent={{-10,10},{10,-10}},
                rotation=90,
                origin={-78,-76})));
          Modelica.Blocks.Math.Division ComputeTau
                                                  "Computes time delay tau from mass flow in stationary model"
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-34,-74})));
          Modelica.Blocks.Sources.RealExpression MassInPipe(y=length*Modelica.Constants.pi*dh*dh*0.25*rho_default)
                                                                                                              "Mass of water in pipe"
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-28,-36})));
        protected
          parameter Modelica.SIunits.Density rho_default=Medium.density_pTX(
              p=Medium.p_default,
              T=Medium.T_default,
              X=Medium.X_default)
            "Default density (e.g., rho_liquidWater = 995, rho_air = 1.2)"
            annotation (Dialog(group="Advanced"));

        initial equation
          assert(homotopyInitialization, "In " + getInstanceName() +
            ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
            level = AssertionLevel.warning);

        equation
          connect(heaLos_a.heatPort, heatPort) annotation (Line(points={{-70,10},{-70,40},
                  {0,40},{0,100}}, color={191,0,0}));
          connect(heaLos_b.heatPort, heatPort) annotation (Line(points={{70,10},{70,40},
                  {0,40},{0,100}}, color={191,0,0}));

          connect(port_a, heaLos_a.port_b)
            annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
          connect(heaLos_a.port_a, senMasFlo.port_a)
            annotation (Line(points={{-60,0},{-50,0}}, color={0,127,255}));
          connect(heaLos_b.port_b, port_b)
            annotation (Line(points={{80,0},{100,0}}, color={0,127,255}));
          connect(senMasFlo.port_b, res.port_a)
            annotation (Line(points={{-30,0},{-20,0}}, color={0,127,255}));
          connect(res.port_b, heaLos_b.port_a) annotation (Line(points={{0,0},{60,0}}, color={0,127,255}));
          connect(zeroConst.y, min1.u1) annotation (Line(points={{-53,-128},{-38,-128},{-38,-106},{-44,-106}}, color={0,0,127}));
          connect(tau.u1, min1.u1) annotation (Line(points={{14,-116},{4,-116},{4,-128},{-38,-128},{-38,-106},{-44,-106}}, color={0,0,127}));
          connect(min1.y, tauRev.u) annotation (Line(points={{-67,-100},{-78,-100},{-78,-88}}, color={0,0,127}));
          connect(tauRev.y, heaLos_a.tau) annotation (Line(points={{-78,-65},{-78,18},{-64,18},{-64,10}}, color={0,0,127}));
          connect(tau.y, heaLos_b.tau) annotation (Line(points={{37,-110},{48,-110},{48,20},{64,20},{64,10}}, color={0,0,127}));
          connect(senMasFlo.m_flow, ComputeTau.u2) annotation (Line(points={{-40,-11},{-40,-62}}, color={0,0,127}));
          connect(MassInPipe.y, ComputeTau.u1) annotation (Line(points={{-28,-47},{-28,-62}}, color={0,0,127}));
          connect(ComputeTau.y, min1.u2) annotation (Line(points={{-34,-85},{-34,-94},{-44,-94}}, color={0,0,127}));
          connect(tau.u2, min1.u2) annotation (Line(points={{14,-104},{-10,-104},{-10,-100},{-34,-100},{-34,-94},{-44,-94}}, color={0,0,127}));
          annotation (
            Line(points={{70,20},{72,20},{72,0},{100,0}}, color={0,127,255}),
            Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{100,100}})),
            Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{100,100}}),
                graphics={
                Rectangle(
                  extent={{-100,40},{100,-40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-100,30},{100,-30}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={0,127,255}),
                Rectangle(
                  extent={{-100,50},{100,40}},
                  lineColor={175,175,175},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Backward),
                Rectangle(
                  extent={{-100,-40},{100,-50}},
                  lineColor={175,175,175},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Backward),
                Polygon(
                  points={{0,100},{40,62},{20,62},{20,38},{-20,38},{-20,62},{-40,62},{0,
                      100}},
                  lineColor={0,0,0},
                  fillColor={238,46,47},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-30,30},{28,-30}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={215,202,187})}),
            Documentation(revisions="<html>
<ul>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">Buildings, #1341</a>.
</li>
<li>
October 20, 2017, by Michael Wetter:<br/>
Replaced model that lumps flow resistance and transport delays
with two separate models, as these are physically distinct processes.
This also avoids one more layer of models.
<br/>
Revised variable names and documentation to follow guidelines.
</li>
<li>
July 4, 2016 by Bram van der Heijde:<br/>
Introduce <code>pipVol</code>.
</li>
<li>
October 10, 2015 by Marcus Fuchs:<br/>
Copy Icon from KUL implementation and rename model.
Replace resistance and temperature delay by an adiabatic pipe.
</li>
<li>
September, 2015 by Marcus Fuchs:<br/>First implementation.
</li>
</ul>
</html>",         info="<html>
<p>
Pipe with heat loss using the time delay based heat losses and plug flow
for the transport delay of the fluid.
</p>
<h4>Implementation</h4>
<p>
The
<code>spatialDistribution</code> operator is used for the temperature wave propagation
through the length of the pipe. This operator is contained in
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlow\">BaseClasses.PlugFlow</a>.
</p>
<p>
This model does not include thermal inertia of the pipe wall.
The wall inertia is implemented in
<a href=\"modelica://Buildings.Fluid.FixedResistances.PlugFlowPipe\">PlugFlowPipe</a>, which uses this model.
<br/>
The removal of the thermal inertia with a mixing volume can be desirable in the
case where mixing volumes are added manually at the pipe junctions.
</p>
<p>
The model
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss\">
PlugFlowHeatLoss</a>
implements a heat loss in design direction, but leaves the enthalpy unchanged
in opposite flow direction. Therefore it is used in front of and behind the time delay.
</p>
<h4>References</h4>
<p>
Full details on the model implementation and experimental validation can be found
in:
</p>
<p>
van der Heijde, B., Fuchs, M., Ribas Tugores, C., Schweiger, G., Sartor, K., Basciotti, D., M&uuml;ller,
D., Nytsch-Geusen, C., Wetter, M. and Helsen, L. (2017).<br/>
Dynamic equation-based thermo-hydraulic pipe model for district heating and cooling systems.<br/>
<i>Energy Conversion and Management</i>, vol. 151, p. 158-169.
<a href=\"https://doi.org/10.1016/j.enconman.2017.08.072\">doi: 10.1016/j.enconman.2017.08.072</a>.</p>
</html>"));
        end PlugFlowCoreNoDelay;

        partial package PartialSimpleMedium_TboundNoError
          "Class that does not stop simulationwhen temperature range is violated, only warns! Inherits from \"PartialSimpleMedium\". ... Medium model with linear dependency of u, h from temperature. All other quantities, especially density, are constant."

          extends Modelica.Media.Interfaces.PartialPureSubstance(final ThermoStates=Modelica.Media.Interfaces.Choices.IndependentVariables.pT, final singleState=true);

          constant SpecificHeatCapacity cp_const
            "Constant specific heat capacity at constant pressure";
          constant SpecificHeatCapacity cv_const
            "Constant specific heat capacity at constant volume";
          constant Density d_const "Constant density";
          constant DynamicViscosity eta_const "Constant dynamic viscosity";
          constant ThermalConductivity lambda_const "Constant thermal conductivity";
          constant VelocityOfSound a_const "Constant velocity of sound";
          constant Temperature T_min "Minimum temperature valid for medium model";
          constant Temperature T_max "Maximum temperature valid for medium model";
          constant Temperature T0=reference_T "Zero enthalpy temperature";
          constant MolarMass MM_const "Molar mass";

          constant FluidConstants[nS] fluidConstants "Fluid constants";

          redeclare record extends ThermodynamicState "Thermodynamic state"
            AbsolutePressure p "Absolute pressure of medium";
            Temperature T "Temperature of medium";
          end ThermodynamicState;

          redeclare replaceable model extends BaseProperties(T(stateSelect=if
                  preferredMediumStates then StateSelect.prefer else StateSelect.default),
              p(stateSelect=if preferredMediumStates then StateSelect.prefer else
                  StateSelect.default)) "Base properties"
          equation
            assert(T >= T_min and T <= T_max, "
Temperature T (= "       + String(T) + " K) is not
in the allowed range ("       + String(T_min) + " K <= T <= " + String(T_max) + " K)
required from medium model \""       + mediumName + "\".
                             ",         AssertionLevel.error);

            // h = cp_const*(T-T0);
            h = specificEnthalpy_pTX(
                    p,
                    T,
                    X);
            u = cv_const*(T - T0);
            d = d_const;
            R = 0;
            MM = MM_const;
            state.T = T;
            state.p = p;
            annotation (Documentation(info="<html>
<p>
This is the most simple incompressible medium model, where
specific enthalpy h and specific internal energy u are only
a function of temperature T and all other provided medium
quantities are assumed to be constant.
Note that the (small) influence of the pressure term p/d is neglected.
</p>
</html>"));
          end BaseProperties;

          redeclare function setState_pTX
            "Return thermodynamic state from p, T, and X or Xi"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[:]=reference_X "Mass fractions";
            output ThermodynamicState state "Thermodynamic state record";
          algorithm
            state := ThermodynamicState(p=p, T=T);
          end setState_pTX;

          redeclare function setState_phX
            "Return thermodynamic state from p, h, and X or Xi"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Specific enthalpy";
            input MassFraction X[:]=reference_X "Mass fractions";
            output ThermodynamicState state "Thermodynamic state record";
          algorithm
            state := ThermodynamicState(p=p, T=T0 + h/cp_const);
          end setState_phX;

          redeclare replaceable function setState_psX
            "Return thermodynamic state from p, s, and X or Xi"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEntropy s "Specific entropy";
            input MassFraction X[:]=reference_X "Mass fractions";
            output ThermodynamicState state "Thermodynamic state record";
          algorithm
            state := ThermodynamicState(p=p, T=Modelica.Math.exp(s/cp_const +
              Modelica.Math.log(reference_T)))
              "Here the incompressible limit is used, with cp as heat capacity";
          end setState_psX;

          redeclare function setState_dTX
            "Return thermodynamic state from d, T, and X or Xi"
            extends Modelica.Icons.Function;
            input Density d "Density";
            input Temperature T "Temperature";
            input MassFraction X[:]=reference_X "Mass fractions";
            output ThermodynamicState state "Thermodynamic state record";
          algorithm
            assert(false,
              "Pressure can not be computed from temperature and density for an incompressible fluid!");
          end setState_dTX;

          redeclare function extends setSmoothState
            "Return thermodynamic state so that it smoothly approximates: if x > 0 then state_a else state_b"
          algorithm
            state := ThermodynamicState(p=Modelica.Media.Common.smoothStep(
                x,
                state_a.p,
                state_b.p,
                x_small), T=Modelica.Media.Common.smoothStep(
                x,
                state_a.T,
                state_b.T,
                x_small));
          end setSmoothState;

          redeclare function extends dynamicViscosity "Return dynamic viscosity"

          algorithm
            eta := eta_const;
          end dynamicViscosity;

          redeclare function extends thermalConductivity
            "Return thermal conductivity"

          algorithm
            lambda := lambda_const;
          end thermalConductivity;

          redeclare function extends pressure "Return pressure"

          algorithm
            p := state.p;
          end pressure;

          redeclare function extends temperature "Return temperature"

          algorithm
            T := state.T;
          end temperature;

          redeclare function extends density "Return density"

          algorithm
            d := d_const;
          end density;

          redeclare function extends specificEnthalpy "Return specific enthalpy"

          algorithm
            h := cp_const*(state.T - T0);
          end specificEnthalpy;

          redeclare function extends specificHeatCapacityCp
            "Return specific heat capacity at constant pressure"

          algorithm
            cp := cp_const;
          end specificHeatCapacityCp;

          redeclare function extends specificHeatCapacityCv
            "Return specific heat capacity at constant volume"

          algorithm
            cv := cv_const;
          end specificHeatCapacityCv;

          redeclare function extends isentropicExponent "Return isentropic exponent"

          algorithm
            gamma := cp_const/cv_const;
          end isentropicExponent;

          redeclare function extends velocityOfSound "Return velocity of sound"

          algorithm
            a := a_const;
          end velocityOfSound;

          redeclare function specificEnthalpy_pTX
            "Return specific enthalpy from p, T, and X or Xi"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input Temperature T "Temperature";
            input MassFraction X[nX] "Mass fractions";
            output SpecificEnthalpy h "Specific enthalpy";
          algorithm
            h := cp_const*(T - T0);
            annotation (Documentation(info="<html>
<p>
This function computes the specific enthalpy of the fluid, but neglects the (small) influence of the pressure term p/d.
</p>
</html>"));
          end specificEnthalpy_pTX;

          redeclare function temperature_phX
            "Return temperature from p, h, and X or Xi"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Specific enthalpy";
            input MassFraction X[nX] "Mass fractions";
            output Temperature T "Temperature";
          algorithm
            T := T0 + h/cp_const;
          end temperature_phX;

          redeclare function density_phX "Return density from p, h, and X or Xi"
            extends Modelica.Icons.Function;
            input AbsolutePressure p "Pressure";
            input SpecificEnthalpy h "Specific enthalpy";
            input MassFraction X[nX] "Mass fractions";
            output Density d "Density";
          algorithm
            d := density(setState_phX(
                    p,
                    h,
                    X));
          end density_phX;

          redeclare function extends specificInternalEnergy
            "Return specific internal energy"
            extends Modelica.Icons.Function;
          algorithm
            //  u := cv_const*(state.T - T0) - reference_p/d_const;
            u := cv_const*(state.T - T0);
            annotation (Documentation(info="<html>
<p>
This function computes the specific internal energy of the fluid, but neglects the (small) influence of the pressure term p/d.
</p>
</html>"));
          end specificInternalEnergy;

          redeclare function extends specificEntropy "Return specific entropy"
            extends Modelica.Icons.Function;
          algorithm
            s := cv_const*Modelica.Math.log(state.T/T0);
          end specificEntropy;

          redeclare function extends specificGibbsEnergy
            "Return specific Gibbs energy"
            extends Modelica.Icons.Function;
          algorithm
            g := specificEnthalpy(state) - state.T*specificEntropy(state);
          end specificGibbsEnergy;

          redeclare function extends specificHelmholtzEnergy
            "Return specific Helmholtz energy"
            extends Modelica.Icons.Function;
          algorithm
            f := specificInternalEnergy(state) - state.T*specificEntropy(state);
          end specificHelmholtzEnergy;

          redeclare function extends isentropicEnthalpy "Return isentropic enthalpy"
          algorithm
            h_is := cp_const*(temperature(refState) - T0);
          end isentropicEnthalpy;

          redeclare function extends isobaricExpansionCoefficient
            "Returns overall the isobaric expansion coefficient beta"
          algorithm
            beta := 0.0;
          end isobaricExpansionCoefficient;

          redeclare function extends isothermalCompressibility
            "Returns overall the isothermal compressibility factor"
          algorithm
            kappa := 0;
          end isothermalCompressibility;

          redeclare function extends density_derp_T
            "Returns the partial derivative of density with respect to pressure at constant temperature"
          algorithm
            ddpT := 0;
          end density_derp_T;

          redeclare function extends density_derT_p
            "Returns the partial derivative of density with respect to temperature at constant pressure"
          algorithm
            ddTp := 0;
          end density_derT_p;

          redeclare function extends density_derX
            "Returns the partial derivative of density with respect to mass fractions at constant pressure and temperature"
          algorithm
            dddX := fill(0, nX);
          end density_derX;

          redeclare function extends molarMass "Return the molar mass of the medium"
          algorithm
            MM := MM_const;
          end molarMass;
        end PartialSimpleMedium_TboundNoError;

        package ConstantPropertyLiquidWater_extendTempRange_noErrors "Water: Simple liquid water medium (incompressible, constant data). No stop of simulatin for violated temperature ranges!"

          //   redeclare record extends FluidConstants
          //   end FluidConstants;

          constant Modelica.Media.Interfaces.Types.Basic.FluidConstants[1]
            simpleWaterConstants(
            each chemicalFormula="H2O",
            each structureFormula="H2O",
            each casRegistryNumber="7732-18-5",
            each iupacName="oxidane",
            each molarMass=0.018015268);

          extends PartialSimpleMedium_TboundNoError(
            mediumName="SimpleLiquidWater",
            cp_const=4183,
            cv_const=4183,
            d_const=995.586,
            eta_const=1.e-3,
            lambda_const=0.598,
            a_const=1484,
            T_min=Modelica.SIunits.Conversions.from_degC(
                                           -10),
            T_max=Modelica.SIunits.Conversions.from_degC(
                                           300),
            T0=273.15,
            MM_const=0.018015268,
            fluidConstants=simpleWaterConstants);

          annotation (Documentation(info="<html>

</html>"));
        end ConstantPropertyLiquidWater_extendTempRange_noErrors;

        model PlugFlowPipeNoDelay "Pipe model with no delay but het loss similar to PlugFlowPipe"
          extends Core.Buildings.Fluid.Interfaces.PartialTwoPortVector;

          constant Boolean homotopyInitialization = true "= true, use homotopy method"
            annotation(HideResult=true);

          parameter Boolean from_dp=false
            "= true, use m_flow = f(dp) else dp = f(m_flow)"
            annotation (Dialog(tab="Advanced"));

          parameter Modelica.SIunits.Length dh=sqrt(4*m_flow_nominal/rho_default/v_nominal/Modelica.Constants.pi)
            "Hydraulic diameter (assuming a round cross section area)"
            annotation (Dialog(group="Material"));

          parameter Modelica.SIunits.Velocity v_nominal = 1.5
            "Velocity at m_flow_nominal (used to compute default value for hydraulic diameter dh)"
            annotation(Dialog(group="Nominal condition"));

          parameter Real ReC=4000
            "Reynolds number where transition to turbulent starts";

          parameter Modelica.SIunits.Height roughness=2.5e-5
            "Average height of surface asperities (default: smooth steel pipe)"
            annotation (Dialog(group="Material"));

          parameter Modelica.SIunits.Length length "Pipe length"
            annotation (Dialog(group="Material"));

          parameter Modelica.SIunits.MassFlowRate m_flow_nominal
            "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

          parameter Modelica.SIunits.MassFlowRate m_flow_small = 1E-4*abs(
            m_flow_nominal) "Small mass flow rate for regularization of zero flow"
            annotation (Dialog(tab="Advanced"));

          parameter Modelica.SIunits.Length dIns
            "Thickness of pipe insulation, used to compute R"
            annotation (Dialog(group="Thermal resistance"));

          parameter Modelica.SIunits.ThermalConductivity kIns
            "Heat conductivity of pipe insulation, used to compute R"
            annotation (Dialog(group="Thermal resistance"));

          parameter Modelica.SIunits.SpecificHeatCapacity cPip=2300
            "Specific heat of pipe wall material. 2300 for PE, 500 for steel"
            annotation (Dialog(group="Material"));

          parameter Modelica.SIunits.Density rhoPip(displayUnit="kg/m3")=930
            "Density of pipe wall material. 930 for PE, 8000 for steel"
            annotation (Dialog(group="Material"));

          parameter Modelica.SIunits.Length thickness = 0.0035
            "Pipe wall thickness"
            annotation (Dialog(group="Material"));

          parameter Modelica.SIunits.Temperature T_start_in(start=Medium.T_default)=
            Medium.T_default "Initialization temperature at pipe inlet"
            annotation (Dialog(tab="Initialization"));
          parameter Modelica.SIunits.Temperature T_start_out(start=Medium.T_default)=
            T_start_in "Initialization temperature at pipe outlet"
            annotation (Dialog(tab="Initialization"));
          parameter Boolean initDelay(start=false)=true
            "Initialize delay for a constant mass flow rate if true, otherwise start from 0"
            annotation (Dialog(tab="Initialization"));
          parameter Modelica.SIunits.MassFlowRate m_flow_start=m_flow_nominal
                                                                 "Initial value of mass flow rate through pipe"
            annotation (Dialog(tab="Initialization", enable=initDelay));

          parameter Real R(unit="(m.K)/W")=1/(kIns*2*Modelica.Constants.pi/
            Modelica.Math.log((dh/2 + thickness + dIns)/(dh/2 + thickness)))
            "Thermal resistance per unit length from fluid to boundary temperature"
            annotation (Dialog(group="Thermal resistance"));

          parameter Real fac=1
            "Factor to take into account flow resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal";

          parameter Boolean linearized = false
            "= true, use linear relation between m_flow and dp for any flow rate"
            annotation(Evaluate=true, Dialog(tab="Advanced"));

          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
            "Heat transfer to or from surroundings (heat loss from pipe results in a positive heat flow)"
            annotation (Placement(transformation(extent={{-10,90},{10,110}})));

          PlugFlowCoreNoDelay cor(
            redeclare final package Medium = Medium,
            final dh=dh,
            final v_nominal=v_nominal,
            final length=length,
            final C=C,
            final R=R,
            final m_flow_small=m_flow_small,
            final m_flow_nominal=m_flow_nominal,
            final T_start_in=T_start_in,
            final T_start_out=T_start_out,
            final m_flow_start=m_flow_start,
            final initDelay=initDelay,
            final from_dp=from_dp,
            final fac=fac,
            final ReC=ReC,
            final thickness=thickness,
            final roughness=roughness,
            final allowFlowReversal=allowFlowReversal,
            final homotopyInitialization=homotopyInitialization,
            final linearized=linearized) "Describing the pipe behavior" annotation (Placement(transformation(extent={{-10,-12},{10,8}})));

          // In the volume, below, we scale down V and use
          // mSenFac. Otherwise, for air, we would get very large volumes
          // which affect the delay of water vapor and contaminants.
          // See also Buildings.Fluid.FixedResistances.Validation.PlugFlowPipes.TransportWaterAir
          // for why mSenFac is 10 and not 1000, as this gives more reasonable
          // temperature step response
          Core.Buildings.Fluid.MixingVolumes.MixingVolume vol(
            redeclare final package Medium = Medium,
            final m_flow_nominal=m_flow_nominal,
            final V=0.01,
            final nPorts=nPorts + 1,
            final T_start=T_start_out,
            final energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
            final mSenFac=if rho_default > 500 then 1 else 10) "Control volume connected to ports_b. Represents equivalent pipe wall thermal capacity."
            annotation (Placement(transformation(extent={{60,20},{80,40}})));

        protected
          parameter Modelica.SIunits.HeatCapacity CPip=
            length*((dh + 2*thickness)^2 - dh^2)*Modelica.Constants.pi/4*cPip*rhoPip "Heat capacity of pipe wall";

          final parameter Modelica.SIunits.Volume VEqu=CPip/(rho_default*cp_default)
            "Equivalent water volume to represent pipe wall thermal inertia";

          parameter Medium.ThermodynamicState sta_default=Medium.setState_pTX(
              T=Medium.T_default,
              p=Medium.p_default,
              X=Medium.X_default) "Default medium state";

          parameter Modelica.SIunits.SpecificHeatCapacity cp_default=
              Medium.specificHeatCapacityCp(state=sta_default)
            "Heat capacity of medium";

          parameter Real C(unit="J/(K.m)")=
            rho_default*Modelica.Constants.pi*(dh/2)^2*cp_default
            "Thermal capacity per unit length of water in pipe";

          parameter Modelica.SIunits.Density rho_default=Medium.density_pTX(
              p=Medium.p_default,
              T=Medium.T_default,
              X=Medium.X_default)
            "Default density (e.g., rho_liquidWater = 995, rho_air = 1.2)"
            annotation (Dialog(group="Advanced"));

        initial equation
          assert(homotopyInitialization, "In " + getInstanceName() +
            ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
            level = AssertionLevel.warning);

        equation
          for i in 1:nPorts loop
            connect(vol.ports[i + 1], ports_b[i])
            annotation (Line(points={{70,20},{72,20},{72,6},{72,0},{100,0}},
                color={0,127,255}));
          end for;
          connect(cor.heatPort, heatPort)
            annotation (Line(points={{0,8},{0,100}},         color={191,0,0}));

          connect(cor.port_b, vol.ports[1])
            annotation (Line(points={{10,-0.333333},{70,-0.333333},{70,20}},
                                                             color={0,127,255}));

          connect(cor.port_a, port_a)
            annotation (Line(points={{-10,-0.333333},{-56,0},{-100,0}},
                                                                color={0,127,255}));
          annotation (
            Line(points={{70,20},{72,20},{72,0},{100,0}}, color={0,127,255}),
            Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                    100,100}})),
            Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                    100}}), graphics={
                Rectangle(
                  extent={{-100,40},{100,-40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-100,30},{100,-30}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={0,127,255}),
                Rectangle(
                  extent={{-100,50},{100,40}},
                  lineColor={175,175,175},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Backward),
                Rectangle(
                  extent={{-100,-40},{100,-50}},
                  lineColor={175,175,175},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Backward),
                Polygon(
                  points={{0,90},{40,62},{20,62},{20,38},{-20,38},{-20,62},{-40,62},{0,
                      90}},
                  lineColor={0,0,0},
                  fillColor={238,46,47},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-30,30},{28,-30}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={215,202,187}),
                Text(
                  extent={{-100,-72},{100,-88}},
                  lineColor={0,0,0},
                  textString="L = %length
d = %dh")}),Documentation(revisions="<html>
<ul>
<li>
April 14, 2020, by Michael Wetter:<br/>
Changed <code>homotopyInitialization</code> to a constant.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1341\">Buildings, #1341</a>.
</li>
<li>
March 6, 2020, by Jelger Jansen:<br/>
Revised calculation of thermal resistance <code>R</code>
by using correct radiuses.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1310\">#1310</a>.
</li>
<li>
October 23, 2017, by Michael Wetter:<br/>
Revised variable names and documentation to follow guidelines.
Corrected malformed hyperlinks.
</li>
<li>
July 4, 2016 by Bram van der Heijde:<br/>
Introduce <code>pipVol</code>.
</li>
<li>
October 10, 2015 by Marcus Fuchs:<br/>
Copy Icon from KUL implementation and rename model.
Replace resistance and temperature delay by an adiabatic pipe.
</li>
<li>September, 2015 by Marcus Fuchs:<br/>
First implementation.
</li>
</ul>
</html>",         info="<html>
<p>
Pipe with heat loss using the time delay based heat losses and transport
of the fluid using a plug flow model, applicable for simulation of long
pipes such as in district heating and cooling systems.</p>
<p>
This model takes into account transport delay along the pipe length idealized
as a plug flow.
The model also includes thermal inertia of the pipe wall.
</p>
<h4>Implementation</h4>
<p>Heat losses are implemented by
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowHeatLoss</a>
at each end of the pipe (see
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore\">
Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore</a>).
Depending on the flow direction, the temperature difference due to heat losses
is subtracted at the right fluid port.
</p>
<p>
The pressure drop is implemented using
<a href=\"modelica://Buildings.Fluid.FixedResistances.HydraulicDiameter\">
Buildings.Fluid.FixedResistances.HydraulicDiameter</a>.
</p>
<p>
The thermal capacity of the pipe wall is implemented as a mixing volume
of the fluid in the pipe, of which the thermal capacity
is equal to that of the pipe wall material.
In addition, this mixing volume allows the hydraulic separation of subsequent pipes.
Thanks to the vectorized implementation of the (design) outlet port,
splits and junctions of pipes can be handled in a numerically efficient way.
<br/>
This mixing volume is not present in the
<a href=\"modelica://Buildings.Fluid.FixedResistances.BaseClasses.PlugFlowCore\">PlugFlowCore</a> model,
which can be used in cases where mixing volumes at pipe junctions need to
be added manually.
</p>
<h4>Assumptions</h4>
<ul>
<li>
Heat losses are for steady-state operation.
</li>
<li>
The axial heat diffusion in the fluid, the pipe wall and the ground are neglected.
</li>
<li>
The boundary temperature is uniform.
</li>
<li>
The thermal inertia of the pipe wall material is lumped on the side of the pipe
that is connected to <code>ports_b</code>.
</li>
</ul>
<h4>References</h4>
<p>
Full details on the model implementation and experimental validation can be found
in:
</p>
<p>
van der Heijde, B., Fuchs, M., Ribas Tugores, C., Schweiger, G., Sartor, K.,
Basciotti, D., M&uuml;ller, D., Nytsch-Geusen, C., Wetter, M. and Helsen, L.
(2017).<br/>
Dynamic equation-based thermo-hydraulic pipe model for district heating and
cooling systems.<br/>
<i>Energy Conversion and Management</i>, vol. 151, p. 158-169.
<a href=\"https://doi.org/10.1016/j.enconman.2017.08.072\">doi:
10.1016/j.enconman.2017.08.072</a>.</p>
</html>"));
        end PlugFlowPipeNoDelay;
      annotation (Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
      end BaseClasses;

      model IsoplusPipeNoDelay "Single Pipe defined by Isoplus catalogue. No time delay: Instantaneous propagation of temperatures!"

        replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater constrainedby Modelica.Media.Interfaces.PartialMedium "Medium in the component"
          annotation (choicesAllMatching=true);

        extends Core.Buildings.Fluid.Interfaces.PartialTwoPortInterface;
        //extends DisHeatLib.BaseClasses.PartialFourPortVectorInterface(
        //  redeclare package Medium1 = Medium,
        //  redeclare paackage Medium2 = Medium,
        //  final m1_flow_nominal = pipeType.m_flow_nominal,
        //  final m2_flow_nominal = pipeType.m_flow_nominal,
        //  final m1_flow_small = m_flow_small,
        //  final m2_flow_small = m_flow_small,
        //  final allowFlowReversal1 = allowFlowReversal,
        //  final allowFlowReversal2 = allowFlowReversal);

        replaceable parameter BaseClasses.BasePipe pipeType "Pipe type"
        annotation (choicesAllMatching=true);

        parameter Modelica.SIunits.MassFlowRate m_flow_nominal=pipeType.m_flow_nominal
          "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));

        parameter Boolean allowFlowReversal = true
          "= false to simplify equations, assuming, but not enforcing, no flow reversal"
          annotation(Dialog(tab="Assumptions"), Evaluate=true);
        parameter Medium.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
          "Small mass flow rate for regularization of zero flow"
          annotation(Dialog(tab = "Advanced"));
        parameter Boolean from_dp = false
          "= true, use m_flow = f(dp) else dp = f(m_flow)"
          annotation (Evaluate=true, Dialog(tab="Flow resistance"));
        parameter Boolean linearizeFlowResistance = false
          "= true, use linear relation between m_flow and dp for any flow rate"
          annotation(Dialog(tab="Flow resistance"));
        // Parameters
        parameter Modelica.SIunits.Length L "Length of the pipes";
        parameter Real ReC=4000
          "Reynolds number where transition to turbulent starts"
          annotation (Evaluate=true, Dialog(group="Additional parameters"));
        parameter Real fac=1
          "Factor to take into account flow resistance of bends etc., fac=dp_nominal/dpStraightPipe_nominal"
          annotation (Evaluate=true, Dialog(group="Additional parameters"));
        parameter Real cf = 1.0 "Correction factor of heat losses (needed for aggregation)"
          annotation (Evaluate=true, Dialog(group="Additional parameters"));

        // Initialization
        parameter Modelica.SIunits.Temperature T_init = Medium.T_default "Initial temperature of supply pipe"
          annotation(Dialog(tab = "Initialization"));

        Modelica.SIunits.Power Q_flow "Heat flow to soil (i.e., losses)";
          Modelica.SIunits.ThermodynamicTemperature T_a "Temperature at port_a";
        Modelica.SIunits.ThermodynamicTemperature T_b "Temperature at port_b";
          Modelica.SIunits.HeatFlowRate Pth_grid  "Heat flow, Pth_grid > 0: fed into grid, Pth_grid < 0: taken from grid";

      protected
        BaseClasses.PlugFlowPipeNoDelay                    pipe_sl(
          redeclare package Medium = Medium,
          from_dp=from_dp,
          ReC=ReC,
          roughness=pipeType.pipeMaterial.roughness,
          m_flow_small=m_flow_small,
          dIns=pipeType.dIns,
          length=L,
          m_flow_nominal=pipeType.m_flow_nominal,
          T_start_in=T_init,
          T_start_out=T_init,
          cPip=pipeType.pipeMaterial.cPip,
          rhoPip=pipeType.pipeMaterial.rhoPip,
          v_nominal=pipeType.v_nominal,
          dh=pipeType.dh,
          thickness=pipeType.dWall,
          initDelay=true,
          m_flow_start=m_flow_nominal,
          fac=fac,
          linearized=linearizeFlowResistance,
          allowFlowReversal=allowFlowReversal,
          kIns=cf*pipeType.kIns,
          nPorts=1)
          annotation (Placement(transformation(extent={{-10,-8},{10,12}})));
      public
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
          annotation (Placement(transformation(extent={{-10,90},{10,110}})));
      equation
        Q_flow = heatPort.Q_flow;

         Pth_grid = if port_a.m_flow > 0 then -abs(port_a.m_flow*(port_b.h_outflow - inStream(port_a.h_outflow))) else -abs(port_a.m_flow*(inStream(port_b.h_outflow) - port_a.h_outflow));
          T_a = if port_a.m_flow < 0 then Medium.temperature_phX(port_a.p, port_a.h_outflow, X = port_a.Xi_outflow) else Medium.temperature_phX(port_a.p, inStream(port_a.h_outflow), X = port_a.Xi_outflow);
          T_b = if port_a.m_flow < 0 then Medium.temperature_phX(port_b.p, inStream(port_b.h_outflow), X = port_b.Xi_outflow) else Medium.temperature_phX(port_b.p, port_b.h_outflow, X = port_b.Xi_outflow);

        connect(pipe_sl.heatPort, heatPort)
          annotation (Line(points={{0,12},{0,100}},        color={191,0,0}));
        connect(port_a, pipe_sl.port_a)
          annotation (Line(points={{-100,0},{-56,0},{-56,2},{-10,2}},
                                                      color={0,127,255}));
        connect(pipe_sl.ports_b[1], port_b)
          annotation (Line(points={{10,2},{56,2},{56,0},{100,0}},
                                                    color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,40},{100,-40}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={255,0,0}),
              Rectangle(
                extent={{-50,-13},{50,-23}},
                lineColor=DynamicSelect({255,255,255}, if port_a.m_flow > 0.001 then {0,0,0} elseif port_a.m_flow < -0.001 then {255,255,255} else {0,220,0}),
                fillPattern=FillPattern.Solid,
                fillColor=DynamicSelect({255,255,255}, if port_a.m_flow > 0.001 then {0,0,0} elseif port_a.m_flow < -0.001 then {255,255,255} else {0,220,0})),
              Polygon(
                points={{40,2},{40,-38},{75,-18}},
                lineColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {0,0,0}),
                lineThickness=0,
                fillPattern=FillPattern.Solid,
                fillColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {0,0,0})),
              Polygon(
                points={{-40,2},{-40,-38},{-75,-18}},
                lineColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {255,255,255}),
                lineThickness=0,
                fillPattern=FillPattern.Solid,
                fillColor=DynamicSelect({255,255,255}, if port_a.m_flow < 0.001 and port_a.m_flow > -0.001 then {0,220,0} else {255,255,255})),
              Text(
                extent={{-58,-4},{58,40}},
                lineColor={255,255,255},
                textString="No Delay!",
                textStyle={TextStyle.Bold})}),
          Documentation(revisions="<html>
<ul>
<li>Feburary 27, 2019, by Benedikt Leitner:<br>Implementation and added User&apos;s guide. </li>
</ul>
</html>"));
      end IsoplusPipeNoDelay;
    end Network;
  end Components;

  package Facilities

    package ConsumerSimple

      model ConsumerSimple
        replaceable package Medium =
            Modelica.Media.Interfaces.PartialMedium "Medium in the component"
            annotation (choicesAllMatching = true);

        outer Modelica.Fluid.System system "System wide properties";
        parameter Modelica.SIunits.TemperatureDifference dT_min=10 "Minimum temperature difference between flow and return";
        parameter Modelica.SIunits.MassFlowRate m_flow_max=100;
        parameter Boolean kW_as_input=false "if true: input in kW, if false: input in W";
        Modelica.SIunits.Power Q_flow_kW;
        Modelica.SIunits.ThermodynamicTemperature T_flow;
        Modelica.SIunits.HeatFlowRate Pth_grid "Heat flow, Pth_grid > 0: fed into grid, Pth_grid < 0: taken from grid";
        Modelica.SIunits.ThermodynamicTemperature T_a "Temperature at port_a";
        Modelica.SIunits.ThermodynamicTemperature T_b "Temperature at port_b";
        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium = Medium)
                                                     annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium = Medium)
                                                     annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Blocks.Interfaces.RealInput Q_flow "Thermal energy demand" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={-50,-120})));
        Modelica.Blocks.Interfaces.RealInput T_r "Return temperature" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={50,-120})));
        Modelica.Blocks.Interfaces.RealOutput out_Q_consume annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,110})));
      equation
        // Massflow computation
        // Compute massflow depending on flow enthalpy and thermal power demand
        // If flow temperature falls below return temperature, the simulation will continue but first law of thermodynamics will be violated (return temperature > flow temperature)

        port_a.m_flow = if kW_as_input then min((Q_flow_kW / (inStream(port_a.h_outflow) - port_b.h_outflow)), m_flow_max) else min((Q_flow / (inStream(port_a.h_outflow) - port_b.h_outflow)), m_flow_max);
        Q_flow_kW = 1000*Q_flow;

        // Mass balance (no storage)
        port_a.m_flow + port_b.m_flow = 0;

        T_flow = Medium.temperature_phX(port_a.p, inStream(port_a.h_outflow), X = port_a.Xi_outflow);

        // Energy balance (no storage)
        // Enthalpy of return is calculated based on return temperature (T_r)
        port_a.h_outflow = inStream(port_a.h_outflow);
        port_b.h_outflow = Medium.specificEnthalpy(
          Medium.setState_pTX(
            p = port_b.p,
            T = min(T_r, T_flow - dT_min),
            X = port_b.Xi_outflow));

      T_a = Medium.temperature_phX(port_a.p, inStream(port_a.h_outflow), X = port_a.Xi_outflow);
      T_b = Medium.temperature_phX(port_b.p, port_b.h_outflow, X = port_a.Xi_outflow);

          Pth_grid = if port_a.m_flow > 0 then port_a.m_flow*(port_b.h_outflow - inStream(port_a.h_outflow)) else port_a.m_flow*(inStream(port_b.h_outflow) - port_a.h_outflow);

        // Output
        out_Q_consume = port_a.m_flow*(inStream(port_a.h_outflow) - port_b.h_outflow);

        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{68,4},{100,-4}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Rectangle(
              extent={{-100,4},{-68,-4}},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
              Rectangle(
                extent={{-72,34},{72,-84}},
                lineColor={255,255,255},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-54,-36},{-24,-86}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineThickness=1,
                pattern=LinePattern.None),
              Polygon(
                points={{-88,32},{0,86},{88,32},{-88,32}},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-6,24},{56,-8}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineThickness=1,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-54,24},{-24,-8}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineThickness=1,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-6,-36},{56,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineThickness=1,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                radius=45,
                lineThickness=0.5),
              Line(
                points={{-102,78},{-30,70},{-52,74},{-44,72},{-58,74},{-70,72}},
                color={238,46,47},
                pattern=LinePattern.None,
                thickness=1)}),                                        Diagram(coordinateSystem(preserveAspectRatio=false)));
      end ConsumerSimple;
    end ConsumerSimple;

    package Producer_pT

      model Producer_pT
        replaceable package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater
           "Medium in the component"
            annotation (choices(
              choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
              choice(redeclare package Medium = Buildings.Media.Water "Water"),
              choice(redeclare package Medium =
                  Buildings.Media.Antifreeze.PropyleneGlycolWater (
                    property_T=293.15,
                    X_a=0.40)
                    "Propylene glycol water, 40% mass fraction")));

         parameter Modelica.SIunits.PressureDifference dp_max=1000000   "Maximum pressuredifference of pump";
         parameter Modelica.SIunits.Pressure p_return=433000
                                                      "Constant pressure on suction side of pump";
         parameter Real eta_pump = 0.6 "Constant efficiency of pump";
            Modelica.SIunits.HeatFlowRate Pth_grid "Heat flow, Pth_grid > 0: fed into grid, Pth_grid < 0: taken from grid";
        Modelica.SIunits.ThermodynamicTemperature T_a "Temperature at port_a";
        Modelica.SIunits.ThermodynamicTemperature T_b "Temperature at port_b";

        Modelica.Blocks.Interfaces.RealOutput Q_gen annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,110}),  iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={110,-30})));
        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium = Medium)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium = Medium)
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Blocks.Interfaces.RealInput in_Tout annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-60,120}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={-60,120})));
        Modelica.Blocks.Interfaces.RealInput in_dp_badpoint annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={50,-120})));
        Modelica.Blocks.Interfaces.RealInput in_dp_target annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={-50,-120}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,120})));
        Modelica.Blocks.Logical.Switch switch1 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={30,-50})));
        Modelica.Blocks.Continuous.LimPID PID(
          controllerType=Modelica.Blocks.Types.SimpleController.P,
          k=10000,
          yMax=dp_max,                                                                                  yMin=0) annotation (Placement(transformation(extent={{40,-90},{60,-70}})));
        Modelica.Blocks.Interfaces.RealOutput P_el_pump annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,110}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={110,-50})));
        Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        Modelica.Blocks.Math.Product P_hydraulic annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=90,
              origin={30,50})));
        Modelica.Blocks.Math.Division division annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,80})));
        Modelica.Blocks.Sources.Constant const(k=eta_pump) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={56,50})));
        Core.Buildings.Fluid.Sources.Boundary_pT bou_sup(
          redeclare package Medium = Medium,
          use_p_in=true,
          use_T_in=true,
          nPorts=2) annotation (Placement(transformation(extent={{-10,10},{10,-10}})));
        Core.Buildings.Fluid.Sources.Boundary_pT bou_ret(
          redeclare package Medium = Medium,
          p=p_return,
          T=313.15,
          nPorts=2)
          annotation (Placement(transformation(extent={{-70,-10},{-90,10}})));
        Modelica.Fluid.Sensors.Pressure pressure(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-24},{-90,-44}})));
        Modelica.Blocks.Math.Add add annotation (Placement(transformation(extent={{-40,-50},{-20,-30}})));
        Modelica.Blocks.Math.Add add1 annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
        Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-30,30},{-50,10}})));
        Modelica.Fluid.Sensors.SpecificEnthalpy specificEnthalpy(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,24},{-90,44}})));
        Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{20,-10},{40,10}})));
        Modelica.Fluid.Sensors.SpecificEnthalpy specificEnthalpy1(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{110,24},{90,44}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=massFlowRate.m_flow*(specificEnthalpy1.h_out - specificEnthalpy.h_out)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,70})));
        Modelica.Blocks.Interfaces.RealInput in_internal_dp_control_enabled annotation (Placement(transformation(extent={{62,98},{102,138}}), iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,120})));
        Modelica.Blocks.Math.RealToBoolean realToBoolean  "true to control internal pressuredifference of producer, else of input signal dp_is" annotation (Placement(transformation(extent={{-90,-86},{-70,
                  -66}})));
        Modelica.Blocks.Logical.Switch switch2 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-90,70})));
        Modelica.Blocks.Math.RealToBoolean realToBoolean1(threshold=10)
                                                          "true to control internal pressuredifference of producer, else of input signal dp_is" annotation (Placement(transformation(extent={{-160,60},{-140,
                  80}})));
        Modelica.Blocks.Sources.RealExpression realExpression1(y=in_Tout - bou_ret.T)                                                   annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-190,70})));
        Modelica.Blocks.Sources.RealExpression realExpression2(y=bou_ret.T + 10)                                                        annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-150,44})));
      equation


      T_a = Medium.temperature_phX(port_a.p, inStream(port_a.h_outflow), X = port_a.Xi_outflow);
      T_b = Medium.temperature_phX(port_b.p, port_b.h_outflow, X = port_a.Xi_outflow);

      Pth_grid = if port_a.m_flow > 0 then port_a.m_flow*(port_b.h_outflow - inStream(port_a.h_outflow)) else port_a.m_flow*(inStream(port_b.h_outflow) - port_a.h_outflow) "Heat flow fed into grid";

        connect(port_b, port_b)
          annotation (Line(points={{100,0},{100,0}}, color={0,127,255}));
        connect(PID.u_m, in_dp_badpoint) annotation (Line(points={{50,-92},{50,-120}}, color={0,0,127}));
        connect(P_hydraulic.u1, volumeFlowRate.V_flow) annotation (Line(points={{36,38},{36,11},{70,11}}, color={0,0,127}));
        connect(P_hydraulic.y, division.u1) annotation (Line(points={{30,61},{30,68},{44,68}}, color={0,0,127}));
        connect(const.y, division.u2) annotation (Line(points={{56,61},{56,61},{56,68}}, color={0,0,127}));
        connect(division.y, P_el_pump) annotation (Line(points={{50,91},{50,110}}, color={0,0,127}));
        connect(bou_ret.ports[1], port_a) annotation (Line(points={{-90,2},{-90,0},{-100,0}}, color={0,127,255}));
        connect(volumeFlowRate.port_b, port_b) annotation (Line(points={{80,0},{100,0}}, color={0,127,255}));
        connect(port_a, pressure.port) annotation (Line(points={{-100,0},{-100,-24}}, color={0,127,255}));
        connect(add.y, switch1.u1) annotation (Line(points={{-19,-40},{0,-40},{0,-42},{18,-42}}, color={0,0,127}));
        connect(pressure.p, add.u1) annotation (Line(points={{-89,-34},{-42,-34}}, color={0,0,127}));
        connect(add1.y, switch1.u3) annotation (Line(points={{-19,-60},{0,-60},{0,-58},{18,-58}}, color={0,0,127}));
        connect(add1.u1, pressure.p) annotation (Line(points={{-42,-54},{-66,-54},{-66,-34},{-89,-34}}, color={0,0,127}));
        connect(PID.y, add1.u2) annotation (Line(points={{61,-80},{70,-80},{70,-66},{-42,-66}}, color={0,0,127}));
        connect(switch1.y, bou_sup.p_in) annotation (Line(points={{41,-50},{50,-50},{50,-20},{-20,-20},{-20,-8},{-12,-8}}, color={0,0,127}));
        connect(relativePressure.port_a, bou_sup.ports[1]) annotation (Line(points={{-30,20},{10,20},{10,-2}}, color={0,127,255}));
        connect(relativePressure.port_b, bou_ret.ports[2]) annotation (Line(points={{-50,20},{-90,20},{-90,-2}}, color={0,127,255}));
        connect(P_hydraulic.u2, relativePressure.p_rel) annotation (Line(points={{24,38},{24,29},{-40,29}}, color={0,0,127}));
        connect(massFlowRate.port_b, volumeFlowRate.port_a) annotation (Line(points={{40,0},{60,0}}, color={0,127,255}));
        connect(massFlowRate.port_a, bou_sup.ports[2]) annotation (Line(points={{20,0},{16,0},{16,2},{10,2}}, color={0,127,255}));
        connect(specificEnthalpy.port, port_a) annotation (Line(points={{-100,24},{-100,0}}, color={0,127,255}));
        connect(specificEnthalpy1.port, port_b) annotation (Line(points={{100,24},{100,0}}, color={0,127,255}));
        connect(realExpression.y, Q_gen) annotation (Line(points={{8.88178e-16,81},{8.88178e-16,91.5},{0,91.5},{0,110}}, color={0,0,127}));
        connect(in_dp_target, add.u2) annotation (Line(points={{-50,-120},{-50,-46},{-42,-46}}, color={0,0,127}));
        connect(PID.u_s, in_dp_target) annotation (Line(points={{38,-80},{-50,-80},{-50,-120}}, color={0,0,127}));
        connect(switch1.u2, realToBoolean.y) annotation (Line(points={{18,-50},{-8,-50},{-8,-76},{-69,-76}}, color={255,0,255}));
        connect(realToBoolean.u, in_internal_dp_control_enabled) annotation (Line(points={{-92,-76},{82,118}},   color={0,0,127}));
        connect(realExpression1.y, realToBoolean1.u) annotation (Line(points={{-179,70},{-162,70}}, color={0,0,127}));
        connect(realToBoolean1.y, switch2.u2) annotation (Line(points={{-139,70},{-102,70}}, color={255,0,255}));
        connect(realExpression2.y, switch2.u3) annotation (Line(points={{-139,44},{-122,44},{-122,62},{-102,62}}, color={0,0,127}));
        connect(switch2.y, bou_sup.T_in) annotation (Line(points={{-79,70},{-26,70},{-26,-4},{-12,-4}}, color={0,0,127}));
        connect(switch2.u1, in_Tout) annotation (Line(points={{-102,78},{-102,120},{-60,120}}, color={0,0,127}));
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                radius=45,
                lineThickness=0.5),
              Line(
                points={{-42,60},{-14,84},{50,74},{88,76}},
                color={0,0,0},
                thickness=0.5,
                smooth=Smooth.Bezier),
              Line(
                points={{-32,62},{-10,76},{38,66},{88,68}},
                color={0,0,0},
                thickness=0.5,
                smooth=Smooth.Bezier),
              Line(
                points={{-100,0},{-84,0},{-84,-60},{-42,-60}},
                color={28,108,200},
                thickness=1),
              Line(
                points={{58,-60},{84,-60},{84,0},{96,0}},
                color={238,46,47},
                thickness=1),
              Polygon(
                points={{-56,-82},{-48,54},{-28,54},{-24,-30},{-12,-30},{-8,0},{72,0},
                    {74,-82},{-56,-82}},
                lineColor={0,0,0},
                lineThickness=0.5,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-80,-50},{-60,-70}},
                lineColor={0,0,0},
                lineThickness=1,
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-60,-60},{-70,-50}},
                color={0,0,0},
                thickness=1),
              Line(
                points={{-60,-60},{-70,-70}},
                color={0,0,0},
                thickness=1),
              Text(
                extent={{-6,-2},{72,-60}},
                lineColor={255,255,255},
                textString="pT")}));
      end Producer_pT;
    end Producer_pT;

    package Producer_QT

      model Producer_QT_eq
         replaceable package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater
           "Medium in the component"
            annotation (choices(
              choice(redeclare package Medium = Buildings.Media.Air "Moist air"),
              choice(redeclare package Medium = Buildings.Media.Water "Water"),
              choice(redeclare package Medium =
                  Buildings.Media.Antifreeze.PropyleneGlycolWater (
                    property_T=293.15,
                    X_a=0.40)
                    "Propylene glycol water, 40% mass fraction")));
                Modelica.SIunits.HeatFlowRate Pth_grid "Heat flow, Pth_grid > 0: fed into grid, Pth_grid < 0: taken from grid";
        Modelica.SIunits.ThermodynamicTemperature T_a "Temperature at port_a";
        Modelica.SIunits.ThermodynamicTemperature T_b "Temperature at port_b";
      //parameter Modelica.SIunits.Pressure p_return=433000;

        Modelica.Blocks.Interfaces.RealInput TSet_Supply annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              rotation=180,
              origin={-104,34}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-34,96})));
        Modelica.Blocks.Interfaces.RealInput Q_gen annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              rotation=180,
              origin={-104,0}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-100,60})));
        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
              Medium)                                 annotation (Placement(transformation(extent={{-110,
                  -10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
              Medium)                                  annotation (Placement(transformation(extent={{90,-10},
                  {110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
        Modelica.Blocks.Interfaces.RealOutput out_Pth_grid
          annotation (Placement(transformation(extent={{90,-30},{110,-10}}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-102})));
        Modelica.Blocks.Sources.RealExpression scaled_QT(y=Pth_grid) "use for pT < 0" annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
      equation
        // Set enthalpy at port_a
        port_a.h_outflow = inStream(port_a.h_outflow);

        // Set enthalpy at port_b
        port_b.h_outflow = if Q_gen > 0
          then Medium.specificEnthalpy(
            Medium.setState_pTX(
              p = port_b.p,
              T = TSet_Supply,
              X = port_b.Xi_outflow))
       else
           inStream(port_a.h_outflow);

        // Mass balance (no storage)
        port_a.m_flow + port_b.m_flow = 0;

        // Massflow computation
        // Compute massflow depending on flow enthalpy and thermal power
        port_b.m_flow = if Q_gen > 0 then -Q_gen/(port_b.h_outflow - port_a.h_outflow) else 0; // inStream(port_a.h_outflow) because enthalpy of medium flowing into port_a
      T_a = Medium.temperature_phX(port_a.p, inStream(port_a.h_outflow), X = port_a.Xi_outflow);


      T_b = Medium.temperature_phX(port_b.p, port_b.h_outflow, X = port_a.Xi_outflow);

         Pth_grid = if port_a.m_flow > 0 then port_a.m_flow*(port_b.h_outflow - inStream(port_a.h_outflow)) else port_a.m_flow*(inStream(port_b.h_outflow) - port_a.h_outflow);

        connect(scaled_QT.y, out_Pth_grid) annotation (Line(points={{61,-20},{100,-20}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                radius=45,
                lineThickness=0.5),
              Line(
                points={{-42,60},{-14,84},{50,74},{88,76}},
                color={0,0,0},
                thickness=0.5,
                smooth=Smooth.Bezier),
              Line(
                points={{-32,62},{-10,76},{38,66},{88,68}},
                color={0,0,0},
                thickness=0.5,
                smooth=Smooth.Bezier),
              Line(
                points={{-100,0},{-84,0},{-84,-60},{-42,-60}},
                color={28,108,200},
                thickness=1),
              Line(
                points={{58,-60},{84,-60},{84,0},{96,0}},
                color={238,46,47},
                thickness=1),
              Polygon(
                points={{-56,-82},{-48,54},{-28,54},{-24,-30},{-12,-30},{-8,0},{72,0},
                    {74,-82},{-56,-82}},
                lineColor={0,0,0},
                lineThickness=0.5,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-12,-2},{66,-60}},
                lineColor={255,255,255},
                textString="QT")}),                                    Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Producer_QT_eq;
    end Producer_QT;
  end Facilities;
annotation (uses(
    IBPSA(version="3.0.0"),
    Modelica(version="3.2.3"),
    MFWLib(version="14")));
end MeFlexWaermeLib;

model two_supplier_system
  MeFlexWaermeLib.Components.Network.IsoplusPipe P_delaying_pipe(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    redeclare MeFlexWaermeLib.Components.Network.BaseClasses.Isoplus.Isoplus_Std_DN300 pipeType,
    L=750) annotation (Placement(transformation(extent={{18,-4},{56,16}})));
  MeFlexWaermeLib.Components.Network.IsoplusPipeNoDelay isoplusPipeNoDelay(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    redeclare MeFlexWaermeLib.Components.Network.BaseClasses.Isoplus.Isoplus_Std_DN200 pipeType,
    L=1) annotation (Placement(transformation(extent={{-60,-52},{-40,-32}})));
  MeFlexWaermeLib.Components.Network.IsoplusPipeNoDelay isoplusPipeNoDelay1(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    redeclare MeFlexWaermeLib.Components.Network.BaseClasses.Isoplus.Isoplus_Std_DN200 pipeType,
    L=1) annotation (Placement(transformation(extent={{-60,36},{-40,56}})));
  MeFlexWaermeLib.Facilities.Producer_pT.Producer_pT S_slack_supplier(redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-92,-58})));
  MeFlexWaermeLib.Facilities.Producer_QT.Producer_QT_eq D_deployed_supplier(redeclare package Medium =
        MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-92,32})));
  MeFlexWaermeLib.Facilities.ConsumerSimple.ConsumerSimple C_consumer(redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
      m_flow_max=40) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={82,-10})));
  MeFlexWaermeLib.Components.Network.IsoplusPipe delayed_pipe_return(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    redeclare MeFlexWaermeLib.Components.Network.BaseClasses.Isoplus.Isoplus_Std_DN300 pipeType,
    L=750) annotation (Placement(transformation(extent={{18,-38},{56,-18}})));
  MeFlexWaermeLib.Components.Network.IsoplusPipeNoDelay isoplusPipeNoDelay2(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    redeclare MeFlexWaermeLib.Components.Network.BaseClasses.Isoplus.Isoplus_Std_DN200 pipeType,
    L=1) annotation (Placement(transformation(extent={{-60,-82},{-40,-62}})));
  MeFlexWaermeLib.Components.Network.IsoplusPipeNoDelay isoplusPipeNoDelay3(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    redeclare MeFlexWaermeLib.Components.Network.BaseClasses.Isoplus.Isoplus_Std_DN200 pipeType,
    L=1) annotation (Placement(transformation(extent={{-60,6},{-40,26}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ground_temperature(T=556.3) annotation (Placement(transformation(extent={{-18,-94},{2,-74}})));
  MeFlexWaermeLib.Components.Network.Node node1(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=3) annotation (Placement(transformation(extent={{-30,-28},{-22,-20}})));
  MeFlexWaermeLib.Components.Network.Node node2(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=2) annotation (Placement(transformation(extent={{60,-28},{68,-20}})));
  MeFlexWaermeLib.Components.Network.Node node3(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=2) annotation (Placement(transformation(extent={{60,6},{68,14}})));
  MeFlexWaermeLib.Components.Network.Node node4(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=2) annotation (Placement(transformation(extent={{-76,-72},{-68,-64}})));
  MeFlexWaermeLib.Components.Network.Node node5(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=2) annotation (Placement(transformation(extent={{-76,-42},{-68,-34}})));
  MeFlexWaermeLib.Components.Network.Node node6(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=2) annotation (Placement(transformation(extent={{-76,46},{-68,54}})));
  MeFlexWaermeLib.Components.Network.Node node7(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=2) annotation (Placement(transformation(extent={{-76,16},{-68,24}})));
  Modelica.Blocks.Sources.RealExpression T_S(y=130 + 273.15) annotation (Placement(transformation(extent={{-92,82},{-112,102}})));
  Modelica.Blocks.Sources.RealExpression T_D(y=50 + 273.15) annotation (Placement(transformation(extent={{-92,62},{-112,82}})));
  Modelica.Blocks.Sources.RealExpression Q_C(y=4e6) annotation (Placement(transformation(extent={{124,-12},{104,8}})));
  Modelica.Blocks.Sources.RealExpression T_R(y=40 + 273.15) annotation (Placement(transformation(extent={{124,-30},{104,-10}})));
  Modelica.Blocks.Sources.RealExpression dP_internal(y=1) annotation (Placement(transformation(extent={{-136,-68},{-116,-48}})));
  Modelica.Blocks.Sources.RealExpression dP_target(y=5e5) annotation (Placement(transformation(extent={{-136,-48},{-116,-28}})));
  Modelica.Blocks.Sources.RealExpression dP_badpoint(y=0.6*1e5) annotation (Placement(transformation(extent={{-136,-96},{-116,-76}})));
  Modelica.Fluid.Vessels.ClosedVolume T_buffer_tank(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=1,
    use_portsData=false,
    V=0.001,
    nPorts=2) annotation (Placement(transformation(extent={{-10,72},{10,92}})));
  Modelica.Blocks.Sources.RealExpression mf_D_0(y=mf_C_0.y*(T_S.y - T_C_0.y)/(T_S.y - T_D.y))
                                                                              annotation (Placement(transformation(extent={{86,-82},{106,-62}})));
  Modelica.Blocks.Sources.RealExpression Q_S_0(y=Q_C.y - Q_D.y) annotation (Placement(transformation(extent={{56,-100},{76,-80}})));
  Modelica.Blocks.Sources.RealExpression mf_S_0(y=mf_C_0.y - mf_D_0.y)          annotation (Placement(transformation(extent={{86,-100},{106,-80}})));
  Modelica.Blocks.Sources.RealExpression mf_C_0(y=Q_C.y/(4183*(T_C_0.y - T_R.y)))
                                                                       annotation (Placement(transformation(extent={{116,-82},{136,-62}})));
  Modelica.Blocks.Sources.RealExpression T_C_0(y=70 + 273.15)                            annotation (Placement(transformation(extent={{56,-82},{76,-62}})));
  Modelica.Blocks.Sources.RealExpression Q_D(y=mf_D_0.y*4183*(T_D.y - T_R.y)) annotation (Placement(transformation(extent={{-136,6},{-116,26}})));
  MeFlexWaermeLib.Components.Network.Node M_mixing_node(
    redeclare package Medium = MeFlexWaermeLib.Components.Network.BaseClasses.ConstantPropertyLiquidWater_extendTempRange_noErrors,
    m_flow_nominal=10,
    nPorts=3) annotation (Placement(transformation(extent={{-23,7},{-9,21}})));
equation
  connect(delayed_pipe_return.port_b, node2.ports[1]) annotation (Line(points={{56,-28},{62,-28},{62,-28.04},{65.2,-28.04}},
                                                                                                                       color={0,127,255}));
  connect(P_delaying_pipe.port_b, node3.ports[1]) annotation (Line(points={{56,6},{62,6},{62,5.96},{65.2,5.96}},     color={0,127,255}));
  connect(delayed_pipe_return.port_a, node1.ports[1]) annotation (Line(points={{18,-28},{-16.8,-28},{-16.8,-28.04},{-24.4,-28.04}},
                                                                                                                          color={0,127,255}));
  connect(isoplusPipeNoDelay3.port_b, node1.ports[2]) annotation (Line(points={{-40,16},{-34,16},{-34,-28.04},{-26,-28.04}},
                                                                                                                           color={0,127,255}));
  connect(isoplusPipeNoDelay2.port_b, node1.ports[3]) annotation (Line(points={{-40,-72},{-27,-72},{-27,-28.04},{-27.6,-28.04}},
                                                                                                                               color={0,127,255}));
  connect(C_consumer.port_a, node3.ports[2]) annotation (Line(points={{82,0},{82,5.96},{62.8,5.96}},                    color={0,127,255}));
  connect(C_consumer.port_b, node2.ports[2]) annotation (Line(points={{82,-20},{82,-28.04},{62.8,-28.04}},    color={0,127,255}));
  connect(S_slack_supplier.port_b, node5.ports[1]) annotation (Line(points={{-92,-48},{-91.6,-48},{-91.6,-42.04},{-70.8,-42.04}},
                                                                                                                              color={0,127,255}));
  connect(isoplusPipeNoDelay.port_a, node5.ports[2]) annotation (Line(points={{-60,-42},{-60,-42.04},{-73.2,-42.04}},           color={0,127,255}));
  connect(S_slack_supplier.port_a, node4.ports[1]) annotation (Line(points={{-92,-68},{-91.6,-68},{-91.6,-72.04},{-70.8,-72.04}},
                                                                                                                              color={0,127,255}));
  connect(isoplusPipeNoDelay2.port_a, node4.ports[2]) annotation (Line(points={{-60,-72},{-60.4,-72},{-60.4,-72.04},{-73.2,-72.04}},
                                                                                                                                 color={0,127,255}));
  connect(D_deployed_supplier.port_a, node7.ports[1]) annotation (Line(points={{-92,22},{-91.4,22},{-91.4,15.96},{-70.8,15.96}},
                                                                                                                             color={0,127,255}));
  connect(D_deployed_supplier.port_b, node6.ports[1]) annotation (Line(points={{-92,42},{-91.4,42},{-91.4,45.96},{-70.8,45.96}},
                                                                                                                             color={0,127,255}));
  connect(delayed_pipe_return.heatPort, P_delaying_pipe.heatPort) annotation (Line(points={{37,-18},{10,-18},{10,16},{37,16}},               color={191,0,0}));
  connect(T_S.y, S_slack_supplier.in_Tout) annotation (Line(points={{-113,92},{-139,92},{-139,-64},{-104,-64}},color={0,0,127}));
  connect(dP_internal.y, S_slack_supplier.in_internal_dp_control_enabled) annotation (Line(points={{-115,-58},{-104,-58}},                      color={0,0,127}));
  connect(T_D.y, D_deployed_supplier.TSet_Supply) annotation (Line(points={{-113,72},{-124.4,72},{-124.4,28.6},{-101.6,28.6}},
                                                                                                                          color={0,0,127}));
  connect(Q_C.y, C_consumer.Q_flow) annotation (Line(points={{103,-2},{98,-2},{98,-5},{94,-5}},    color={0,0,127}));
  connect(T_R.y, C_consumer.T_r) annotation (Line(points={{103,-20},{98,-20},{98,-15},{94,-15}},
                                                                                              color={0,0,127}));
  connect(dP_target.y, S_slack_supplier.in_dp_target) annotation (Line(points={{-115,-38},{-103,-38},{-103,-52},{-104,-52}},
                                                                                                                          color={0,0,127}));
  connect(dP_badpoint.y, S_slack_supplier.in_dp_badpoint) annotation (Line(points={{-115,-86},{-66.75,-86},{-66.75,-53},{-80,-53}},
                                                                                                                             color={0,0,127}));
  connect(T_buffer_tank.ports[1], P_delaying_pipe.port_a) annotation (Line(points={{-2,72},{2,72},{2,6},{18,6}},
                                                                                                     color={0,127,255}));
  connect(Q_D.y, D_deployed_supplier.Q_gen) annotation (Line(points={{-115,16},{-98,16},{-98,22}}, color={0,0,127}));
  connect(M_mixing_node.ports[1], isoplusPipeNoDelay1.port_b) annotation (Line(points={{-13.2,6.93},{-18.9,6.93},{-18.9,6},{-26,6},{-26,46},{-40,46}},color={0,127,255}));
  connect(M_mixing_node.ports[2], T_buffer_tank.ports[2]) annotation (Line(points={{-16,6.93},{-16,6},{-2,6},{-2,72},{2,72}},
                                                                                                                  color={0,127,255}));
  connect(M_mixing_node.ports[3], isoplusPipeNoDelay.port_b) annotation (Line(points={{-18.8,6.93},{-17,6.93},{-17,-42},{-40,-42}},   color={0,127,255}));
  connect(ground_temperature.port, delayed_pipe_return.heatPort) annotation (Line(points={{2,-84},{10,-84},{10,-18},{37,-18}}, color={191,0,0}));
  connect(ground_temperature.port, isoplusPipeNoDelay2.heatPort) annotation (Line(points={{2,-84},{10,-84},{10,-62},{-50,-62}}, color={191,0,0}));
  connect(isoplusPipeNoDelay1.heatPort, P_delaying_pipe.heatPort) annotation (Line(points={{-50,56},{10,56},{10,16},{37,16}}, color={191,0,0}));
  connect(isoplusPipeNoDelay3.heatPort, P_delaying_pipe.heatPort) annotation (Line(points={{-50,26},{10,26},{10,16},{37,16}}, color={191,0,0}));
  connect(isoplusPipeNoDelay.heatPort, delayed_pipe_return.heatPort) annotation (Line(points={{-50,-32},{10,-32},{10,-18},{37,-18}}, color={191,0,0}));
  connect(node6.ports[2], isoplusPipeNoDelay1.port_a) annotation (Line(points={{-73.2,45.96},{-66,45.96},{-66,46},{-60,46}}, color={0,127,255}));
  connect(node7.ports[2], isoplusPipeNoDelay3.port_a) annotation (Line(points={{-73.2,15.96},{-66,15.96},{-66,16},{-60,16}}, color={0,127,255}));
  annotation (uses(Modelica(version="3.2.3")), experiment(StopTime=86400, __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-160,-100},{260,120}}), graphics={
        Rectangle(extent={{50,-46},{140,-98}}, lineColor={0,0,0},
          lineThickness=0.5),
        Text(
          extent={{68,-48},{118,-54}},
          lineColor={28,108,200},
          textString="Supplemental Values"),
        Bitmap(extent={{78,46},{78,46}}, fileName=""),
        Bitmap(
          extent={{20,8},{148,138}},
          imageSource=
              "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"
               +
              "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",
          fileName="modelica://two_supplier_system/Simulation_parameters.png"),
        Text(
          extent={{-108,18},{-72,6}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          textString="D_deployed_supplier",
          lineColor={28,108,200}),
        Text(
          extent={{-108,-70},{-74,-82}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={28,108,200},
          textString="S_slack_supplier"),
        Rectangle(extent={{-142,110},{16,64}},lineColor={0,0,0},
          lineThickness=0.5),
        Text(
          extent={{-25,4},{25,-4}},
          lineColor={28,108,200},
          origin={-65,106},
          rotation=180,
          textString="Variable Parameters"),
        Text(
          extent={{-88,74},{-76,68}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          textString="in Kelvin",
          lineColor={0,0,0}),
        Text(
          extent={{-88,94},{-76,88}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          textString="in Kelvin",
          lineColor={0,0,0}),
        Text(
          extent={{68,-26},{102,-38}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={28,108,200},
          textString="C_consumer"),
        Text(
          extent={{162,62},{344,-36}},
          lineColor={28,108,200},
          textString="Plot Variables

Temperature:
D_deployed_supplier.T_b
S_slack_supplier.T_b
C_consumer.T_a

Thermal Power:
D_deployed_supplier.Q_gen
S_slack_supplier.Q_gen
C_consumer.Q_flow",
          horizontalAlignment=TextAlignment.Left),
        Rectangle(
          extent={{156,66},{258,-42}},
          lineColor={0,0,0},
          lineThickness=0.5)}),
    Icon(coordinateSystem(extent={{-160,-100},{260,120}})));
end two_supplier_system;
