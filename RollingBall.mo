within ;
package RollingBall "3d ball on an inclined board"
  package Components
    model Board
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation (
          Placement(transformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={0,-98})));
      Modelica.Mechanics.MultiBody.Parts.BodyBox board(
        lengthDirection(displayUnit="1") = {1,0,0},
        width=board.length,
        r={3,0,0},
        color={208,165,64},
        length=Modelica.Math.Vectors.length(board.r),
        widthDirection(displayUnit="1") = {0,1,0},
        height=0.01,
        density(displayUnit="kg/m3") = 400,
        r_shape=-board.r/2,
        animation=true)
        annotation (Placement(transformation(extent={{46,-8},{66,12}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revoluteY(
        phi(fixed=false, start=0),
        w(fixed=true, start=0),
        n={0,1,0},
        useAxisFlange=true,
        cylinderDiameter=0.012) annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=-90,
            origin={0,-22})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a a_X
        annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b b_X
        annotation (Placement(transformation(extent={{-110,-92},{-90,-72}})));
      Modelica.Mechanics.MultiBody.Joints.Revolute revoluteX(
        useAxisFlange=true,
        n(displayUnit="1") = {1,0,0},
        cylinderDiameter=0.012) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,-56})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a a_Y
        annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b b_Y
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation (
          Placement(transformation(
            extent={{-16,-16},{16,16}},
            rotation=90,
            origin={0,98})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation board_width(animation
          =false, r={0,0,board.height}) annotation (Placement(transformation(
            extent={{-14,-14},{14,14}},
            rotation=90,
            origin={0,70})));
    equation
      connect(frame_a, frame_a) annotation (Line(
          points={{0,-98},{0,-98}},
          color={95,95,95},
          thickness=0.5));
      connect(revoluteY.frame_a, revoluteX.frame_b) annotation (Line(
          points={{0,-32},{0,-46}},
          color={95,95,95},
          thickness=0.5));
      connect(revoluteX.frame_a, frame_a) annotation (Line(
          points={{0,-66},{0,-98}},
          color={95,95,95},
          thickness=0.5));
      connect(revoluteY.axis, a_Y) annotation (Line(points={{-10,-22},{-44,-22},
              {-44,40},{-100,40}}, color={0,0,0}));
      connect(revoluteY.support, b_Y) annotation (Line(points={{-10,-28},{-54,
              -28},{-54,0},{-100,0}}, color={0,0,0}));
      connect(revoluteX.axis, a_X) annotation (Line(points={{-10,-56},{-54,-56},
              {-54,-40},{-100,-40}}, color={0,0,0}));
      connect(revoluteX.support, b_X) annotation (Line(points={{-10,-62},{-54,
              -62},{-54,-82},{-100,-82}}, color={0,0,0}));
      connect(revoluteY.frame_b, board.frame_a) annotation (Line(
          points={{0,-12},{0,2},{46,2}},
          color={95,95,95},
          thickness=0.5));
      connect(frame_b, board_width.frame_b) annotation (Line(
          points={{0,98},{0,93},{2.66454e-15,93},{2.66454e-15,84}},
          color={95,95,95},
          thickness=0.5));
      connect(board_width.frame_a, board.frame_a) annotation (Line(
          points={{8.88178e-16,56},{8.88178e-16,2},{46,2}},
          color={95,95,95},
          thickness=0.5));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=10));
    end Board;

    package Ball

      model Ball
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation (
            Placement(transformation(
              extent={{-16,-16},{16,16}},
              rotation=90,
              origin={0,-98})));
        Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticX(useAxisFlange=
              false,                                             animation=false)
          annotation (Placement(transformation(extent={{-48,-28},{-28,-8}})));
        Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticY(
          useAxisFlange=false,                                   animation=false,
                                                                 n(displayUnit=
                "1") = {0,1,0})
          annotation (Placement(transformation(extent={{-12,-28},{8,-8}})));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation ball_radius(animation=
             false, r={0,0,ball.sphereDiameter/2})
                                         annotation (Placement(transformation(
              extent={{-13,-13},{13,13}},
              rotation=90,
              origin={-1,-77})));
        Modelica.Mechanics.MultiBody.Joints.Spherical spherical annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={60,-6})));
        Modelica.Mechanics.MultiBody.Sensors.RelativeVelocity relVel annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-4,-46})));
        Modelica.Mechanics.MultiBody.Visualizers.FixedArrow fixedArrow(
          r_tail={0,0,0},
          n(displayUnit="1") = absoluteAngularVelocity_b.w,
          length=0.3,
          diameter=0.01)
          annotation (Placement(transformation(extent={{80,-32},{100,-12}})));
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity
          absoluteAngularVelocity_b(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)
                                    annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-10,18})));
        Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce_a annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={60,46})));
        Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce_joint
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={32,-18})));
        IdealRolling.IdealRolling idealRolling3_1
          annotation (Placement(transformation(extent={{-26,78},{-6,98}})));
        Modelica.Blocks.Interfaces.RealInput alpha_x
          annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
        Modelica.Blocks.Interfaces.RealInput alpha_y
          annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
        Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedFrame(
          length=0.2,
          diameter=0.01,
          color_x={0,128,0}) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={10,50})));
        Modelica.Mechanics.MultiBody.Forces.WorldTorque torque
          annotation (Placement(transformation(extent={{8,72},{28,92}})));
        Modelica.Mechanics.MultiBody.Parts.Body ball_v2(
          r_CM={0,0,0},
          m=1,
          I_11=0.01,
          I_22=0.01,
          I_33=0.01,
          v_0(fixed=true),
          sphereDiameter=0.2)
          annotation (Placement(transformation(extent={{70,72},{90,92}})));
      equation
        connect(frame_a, ball_radius.frame_a) annotation (Line(
            points={{0,-98},{0,-90},{-1,-90}},
            color={95,95,95},
            thickness=0.5));
        connect(prismaticX.frame_a, ball_radius.frame_b) annotation (Line(
            points={{-48,-18},{-48,-64},{-1,-64}},
            color={95,95,95},
            thickness=0.5));
        connect(frame_a, frame_a) annotation (Line(
            points={{0,-98},{6,-98},{6,-98},{0,-98}},
            color={95,95,95},
            thickness=0.5));
        connect(relVel.frame_a, ball_radius.frame_b) annotation (Line(
            points={{-14,-46},{-48,-46},{-48,-64},{-1,-64}},
            color={95,95,95},
            thickness=0.5));
        connect(prismaticY.frame_a, prismaticX.frame_b) annotation (Line(
            points={{-12,-18},{-28,-18}},
            color={95,95,95},
            thickness=0.5));
        connect(relVel.frame_b, spherical.frame_a) annotation (Line(
            points={{6,-46},{60,-46},{60,-16}},
            color={95,95,95},
            thickness=0.5));
        connect(fixedArrow.frame_a, spherical.frame_a) annotation (Line(
            points={{80,-22},{60,-22},{60,-16}},
            color={95,95,95},
            thickness=0.5));
        connect(cutForce_a.frame_a, spherical.frame_b) annotation (Line(
            points={{60,36},{60,4}},
            color={95,95,95},
            thickness=0.5));
        connect(absoluteAngularVelocity_b.frame_a, spherical.frame_b)
          annotation (Line(
            points={{0,18},{60,18},{60,4}},
            color={95,95,95},
            thickness=0.5));
        connect(prismaticY.frame_b, cutForce_joint.frame_a) annotation (Line(
            points={{8,-18},{22,-18}},
            color={95,95,95},
            thickness=0.5));
        connect(cutForce_joint.frame_b, spherical.frame_a) annotation (Line(
            points={{42,-18},{60,-18},{60,-16}},
            color={95,95,95},
            thickness=0.5));
        connect(relVel.v_rel, idealRolling3_1.vrel) annotation (Line(points={{-4,-57},
                {-66,-57},{-66,94},{-28,94}},         color={0,0,127}));
        connect(alpha_x, idealRolling3_1.alpha_x) annotation (Line(points={{
                -120,80},{-76,80},{-76,88},{-28,88}}, color={0,0,127}));
        connect(alpha_y, idealRolling3_1.alpha_y) annotation (Line(points={{-120,40},
                {-72,40},{-72,82},{-28,82}},          color={0,0,127}));
        connect(idealRolling3_1.tau, torque.torque)
          annotation (Line(points={{-5,82},{6,82}},  color={0,0,127}));
        connect(torque.frame_b, ball_v2.frame_a) annotation (Line(
            points={{28,82},{70,82}},
            color={95,95,95},
            thickness=0.5));
        connect(cutForce_a.frame_b, ball_v2.frame_a) annotation (Line(
            points={{60,56},{60,82},{70,82}},
            color={95,95,95},
            thickness=0.5));
        connect(fixedFrame.frame_a, ball_v2.frame_a) annotation (Line(
            points={{20,50},{34,50},{34,64},{56,64},{56,82},{70,82}},
            color={95,95,95},
            thickness=0.5));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=10, __Dymola_Algorithm="Cvode"));
      end Ball;
    end Ball;

    package IdealRolling

      block IdealRolling
        //extends Modelica.Blocks.Interfaces.MISO(nin=3);
        parameter Real r[3]={0,0,0.1};
        parameter Real Fg[3] = {0,0,1*9.81};
        Modelica.Blocks.Interfaces.RealInput vrel[3]
          annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
        Modelica.Blocks.Interfaces.RealOutput w[3](start={0,0,0}) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={110,60})));
        Modelica.Blocks.Interfaces.RealOutput tau[3](start={0,0,0}) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={110,-60})));
        Modelica.Blocks.Interfaces.RealInput alpha_x
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealInput alpha_y
          annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
        Real Rx[3,3] = {{1,0,0},{0,cos(alpha_x),-sin(alpha_x)},{0,sin(alpha_x),cos(alpha_x)}};
        Real Ry[3,3] = {{cos(alpha_y),0,sin(alpha_y)},{0,1,0},{-sin(alpha_y),0,cos(alpha_y)}};
      equation
        vrel = cross(w,r);
        tau = cross(Fg,Rx*Ry*r);
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}})),           Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-100,-100},{98,-160}},
                lineColor={28,108,200},
                textString="%name"),
              Ellipse(extent={{-40,40},{40,-40}}, lineColor={28,108,200}),
              Line(points={{-60,-40},{60,-40}}, color={28,108,200})}));
      end IdealRolling;
    end IdealRolling;

  end Components;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model RollingBallOnSlopeBoard
      extends Modelica.Icons.Example;
      Components.Board board
        annotation (Placement(transformation(extent={{36,-2},{68,30}})));
      Modelica.Mechanics.MultiBody.Parts.Fixed fixed
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={52,-88})));
      inner Modelica.Mechanics.MultiBody.World world(
        label2="z",
        n={0,0,-1},
        animateWorld=false,
        animateGravity=false)
        annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
      Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(
        animateSphere=false,
        r={0,0,0.6},
        m=1,
        shapeType="cone",
        length=0.6,
        width=0.3,
        height=0.2,
        extra=0.3,
        sphereDiameter=0.05) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={52,-60})));
      Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape(
        shapeType="cylinder",
        r_shape=-{0,0.3,0}/2,
        lengthDirection={0,0.3,0},
        length=0.3,
        width=0.05,
        height=0.05,
        color={100,100,100})
        annotation (Placement(transformation(extent={{76,-54},{96,-34}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r={0,0,
            0.025}) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={52,-26})));
      Modelica.Mechanics.Rotational.Sources.Position positionY(useSupport=true)
        annotation (Placement(transformation(extent={{-60,18},{-40,38}})));
      Modelica.Mechanics.Rotational.Sources.Position positionX(useSupport=true)
        annotation (Placement(transformation(extent={{-56,-28},{-36,-8}})));
      Modelica.Blocks.Sources.Ramp rampX(
        height=2*3.1415/180,
        duration=3,
        startTime=2.5)
        annotation (Placement(transformation(extent={{-98,-32},{-78,-12}})));
      Modelica.Blocks.Sources.Ramp rampY(
        height=2*3.1415/180,
        duration=5,
        startTime=0.5)
        annotation (Placement(transformation(extent={{-96,18},{-76,38}})));
      Components.Ball.Ball ball
        annotation (Placement(transformation(extent={{36,40},{68,72}})));
    equation
      connect(fixedShape.frame_a,bodyShape. frame_b) annotation (Line(
          points={{76,-44},{52,-44},{52,-50}},
          color={95,95,95},
          thickness=0.5));
      connect(bodyShape.frame_b,fixedTranslation. frame_a) annotation (Line(
          points={{52,-50},{52,-36}},
          color={95,95,95},
          thickness=0.5));
      connect(board.frame_a,fixedTranslation. frame_b) annotation (Line(
          points={{52,-1.68},{52,-16}},
          color={95,95,95},
          thickness=0.5));
      connect(bodyShape.frame_a,fixed. frame_b) annotation (Line(
          points={{52,-70},{52,-78}},
          color={95,95,95},
          thickness=0.5));
      connect(positionY.flange, board.a_Y) annotation (Line(points={{-40,28},{
              -6,28},{-6,20.4},{36,20.4}}, color={0,0,0}));
      connect(positionY.support, board.b_Y) annotation (Line(points={{-50,18},{
              -14,18},{-14,14},{36,14}}, color={0,0,0}));
      connect(positionX.flange, board.a_X) annotation (Line(points={{-36,-18},{
              -18,-18},{-18,-6},{4,-6},{4,7.6},{36,7.6}}, color={0,0,0}));
      connect(positionX.support, board.b_X) annotation (Line(points={{-46,-28},
              {-10,-28},{-10,0.88},{36,0.88}}, color={0,0,0}));
      connect(rampX.y, positionX.phi_ref) annotation (Line(points={{-77,-22},{
              -68,-22},{-68,-18},{-58,-18}}, color={0,0,127}));
      connect(board.frame_b, ball.frame_a) annotation (Line(
          points={{52,29.68},{52,40.32}},
          color={95,95,95},
          thickness=0.5));
      connect(rampY.y, positionY.phi_ref)
        annotation (Line(points={{-75,28},{-62,28}}, color={0,0,127}));
      connect(ball.alpha_x, rampX.y) annotation (Line(points={{32.8,68.8},{
              -66.6,68.8},{-66.6,-22},{-77,-22}}, color={0,0,127}));
      connect(ball.alpha_y, rampY.y) annotation (Line(points={{32.8,62.4},{
              -73.6,62.4},{-73.6,28},{-75,28}}, color={0,0,127}));
      annotation (experiment(StopTime=5.5));
    end RollingBallOnSlopeBoard;
  end Examples;

  annotation (uses(Modelica(version="3.2.3"), SeeSaws(version="1")));
end RollingBall;
