within VehicleDynamics.Vehicle.Chassis.Suspension.Experiments;
model Kinematics2_RL
  import Modelica.SIunits;

  parameter SIunits.Position contact_patch[3] = {-1.5494, 0.609600, 0} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_fore_i[3] = {-1.298905, 0.282999, 0.217500} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_aft_i[3] = {-1.490977, 0.282999, 0.217500} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_o[3] = {-1.574797, 0.554998, 0.289560} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position tie_i[3] = {-1.428059, 0.282999, 0.177800} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position tie_o[3] = {-1.462710, 0.587375, 0.240741} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_fore_i[3] = {-1.298905, 0.282999, 0.090000} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_aft_i[3] = {-1.490977, 0.282999, 0.090000} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_o[3] = {-1.554983, 0.579999, 0.113030} annotation(
    Dialog(group = "Geometry"));

 // inputs to the FMU
  input Real rack_input;
  input Real jounce;
 //check the frame for toe, whether to correct frame is used in its resolve ***
 // outputs of the FMU
 output Modelica.SIunits.Angle camber;
 output Modelica.SIunits.Angle toe;


// defining the up vector and the forward vector from the wheel, they stay constant
 parameter Real wheel_up_const[3] = {0, 0, 1};
 parameter Real wheel_fwd_const[3] = {1, 0, 0};

// wheel vectors from the contact patch
 Real wheel_up[3];
 Real wheel_fwd[3];
 //roll
  //python, negative and positive jounce, take the result, left and right half, redefine camber with respect to that plane
  //toe is read from the contact patch frame, so whether it's from jounce or steering will depened on whether either is 0 or both active.
  DoubleWishboneBase doubleWishboneBase(upper_fore_i = upper_fore_i, upper_aft_i = upper_aft_i, lower_fore_i = lower_fore_i, lower_aft_i = lower_aft_i, upper_o = upper_o, lower_o = lower_o, tie_i = tie_i, tie_o = tie_o, contact_patch = contact_patch) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-110, -110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = upper_fore_i, animation = false) annotation(
    Placement(transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = upper_aft_i, animation = false) annotation(
    Placement(transformation(origin = {70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(r = lower_fore_i, animation = false) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed3(r = lower_aft_i, animation = false) annotation(
    Placement(transformation(origin = {70, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed4(r = tie_i, animation = false) annotation(
    Placement(transformation(origin = {70, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange = true, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {40, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
    Placement(transformation(origin = {-10, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed5(r = contact_patch, animation = false) annotation(
    Placement(transformation(origin = {-80, 86}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Planar planar annotation(
    Placement(transformation(origin = {-80, 56}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic1(useAxisFlange = true, n = {0, 0, 1})  annotation(
    Placement(transformation(origin = {-81, -21}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position position1 annotation(
    Placement(transformation(origin = {-95, -43}, extent = {{-7, -7}, {7, 7}})));
 Modelica.Mechanics.MultiBody.Joints.Revolute revolute(n = {1, 0, 0})  annotation(
    Placement(transformation(origin = {-80, 30}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));
 Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(n = {0, 1, 0})  annotation(
    Placement(transformation(origin = {-81, 3}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
 Modelica.Blocks.Sources.RealExpression realExpression1(y = rack_input)  annotation(
    Placement(transformation(origin = {-60, -72}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Sources.RealExpression realExpression11(y = jounce) annotation(
    Placement(transformation(origin = {-124, -44}, extent = {{-10, -10}, {10, 10}})));
equation
// using resolve2 to get the local vectors into the world frame
  wheel_up = Modelica.Mechanics.MultiBody.Frames.resolve2(doubleWishboneBase.contact_patch_frame.R, wheel_up_const);
  wheel_fwd = Modelica.Mechanics.MultiBody.Frames.resolve2(doubleWishboneBase.contact_patch_frame.R, wheel_fwd_const);
// calculating the camber and toe
  camber = atan2(wheel_up[2], wheel_up[3]);
  toe = -atan2(wheel_fwd[2], wheel_fwd[1]);
  connect(fixed.frame_b, doubleWishboneBase.upper_fore_i_frame) annotation(
    Line(points = {{60, 80}, {0, 80}, {0, 10}}, color = {95, 95, 95}));
  connect(fixed1.frame_b, doubleWishboneBase.upper_aft_i_frame) annotation(
    Line(points = {{60, 40}, {6, 40}, {6, 10}}, color = {95, 95, 95}));
  connect(fixed3.frame_b, doubleWishboneBase.lower_aft_i_frame) annotation(
    Line(points = {{60, -40}, {30, -40}, {30, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(fixed2.frame_b, doubleWishboneBase.lower_fore_i_frame) annotation(
    Line(points = {{60, 0}, {40, 0}, {40, 6}, {10, 6}}, color = {95, 95, 95}));
  connect(prismatic.frame_a, fixed4.frame_b) annotation(
    Line(points = {{50, -80}, {60, -80}}, color = {95, 95, 95}));
  connect(position.flange, prismatic.axis) annotation(
    Line(points = {{0, -60}, {32, -60}, {32, -74}}, color = {0, 127, 0}));
  connect(prismatic.frame_b, doubleWishboneBase.tie_i_frame) annotation(
    Line(points = {{30, -80}, {20, -80}, {20, -6}, {10, -6}}, color = {95, 95, 95}));
  connect(position1.flange, prismatic1.axis) annotation(
    Line(points = {{-88, -43}, {-77, -43}, {-77, -27}}, color = {0, 127, 0}));
  connect(prismatic1.frame_b, doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{-81, -28}, {-81, -32}, {0, -32}, {0, -10}}, color = {95, 95, 95}));
  connect(planar.frame_a, fixed5.frame_b) annotation(
    Line(points = {{-80, 66}, {-80, 76}}, color = {95, 95, 95}));
  connect(revolute.frame_a, planar.frame_b) annotation(
    Line(points = {{-80, 38}, {-80, 46}}, color = {95, 95, 95}));
  connect(revolute.frame_b, revolute1.frame_a) annotation(
    Line(points = {{-80, 22}, {-80, 10}}, color = {95, 95, 95}));
  connect(revolute1.frame_b, prismatic1.frame_a) annotation(
    Line(points = {{-80, -4}, {-80, -14}}, color = {95, 95, 95}));
  connect(realExpression1.y, position.s_ref) annotation(
    Line(points = {{-48, -72}, {-30, -72}, {-30, -60}, {-22, -60}}, color = {0, 0, 127}));
 connect(realExpression11.y, position1.s_ref) annotation(
    Line(points = {{-113, -44}, {-108.5, -44}, {-108.5, -42}, {-104, -42}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 2, Tolerance = 1e-06, Interval = 0.0004),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,evaluateAllParameters,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"),
    Diagram(coordinateSystem(extent = {{-140, 100}, {300, -140}})));
end Kinematics2_RL;
