% Simscape(TM) Multibody(TM) version: 25.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(5).translation = [0.0 0.0 0.0];
smiData.RigidTransform(5).angle = 0.0;
smiData.RigidTransform(5).axis = [0.0 0.0 0.0];
smiData.RigidTransform(5).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [5.0009999999999994 -0.16630841243393565 10.000000000000002];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962573 -0.57735026918962584 0.57735026918962573];
smiData.RigidTransform(1).ID = "B[Part2^Assem1-2:-:Part3^Assem1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [1.4543921622589551e-14 2.5009999999999821 5.0000000000000258];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962584 -0.57735026918962595 0.57735026918962562];
smiData.RigidTransform(2).ID = "F[Part2^Assem1-2:-:Part3^Assem1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 55 0];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(3).ID = "B[Part1^Assem1-1:-:Part2^Assem1-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-1.1102230246251565e-16 -0.16630841243393757 5.0000000000000009];  % mm
smiData.RigidTransform(4).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(4).axis = [1 -3.059426462190721e-33 -5.2090452783293039e-17];
smiData.RigidTransform(4).ID = "F[Part1^Assem1-1:-:Part2^Assem1-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0 0 0];  % mm
smiData.RigidTransform(5).angle = 0;  % rad
smiData.RigidTransform(5).axis = [0 0 0];
smiData.RigidTransform(5).ID = "RootGround[Part1^Assem1-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(3).mass = 0.0;
smiData.Solid(3).CoM = [0.0 0.0 0.0];
smiData.Solid(3).MoI = [0.0 0.0 0.0];
smiData.Solid(3).PoI = [0.0 0.0 0.0];
smiData.Solid(3).color = [0.0 0.0 0.0];
smiData.Solid(3).opacity = 0.0;
smiData.Solid(3).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.0036391723656919966;  % kg
smiData.Solid(1).CoM = [-2.5614402016710604e-05 -2.5919884605319035 25.066966943807603];  % mm
smiData.Solid(1).MoI = [0.69264935763744284 0.69282377082487412 0.044077321136424513];  % kg*mm^2
smiData.Solid(1).PoI = [-0.0066164393040586723 -2.8049906128707978e-07 -5.9916142969337309e-07];  % kg*mm^2
smiData.Solid(1).color = [0.89411764705882346 0.89411764705882346 0.89411764705882346];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Part3^Assem1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.0040251655874119201;  % kg
smiData.Solid(2).CoM = [0 25.670731707317074 0.16225210969164502];  % mm
smiData.Solid(2).MoI = [0.91546118198694915 0.049396830507199778 0.91545853284966705];  % kg*mm^2
smiData.Solid(2).PoI = [0.00043804924954717631 0 0];  % kg*mm^2
smiData.Solid(2).color = [0.89411764705882346 0.89411764705882346 0.89411764705882346];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Part1^Assem1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.00052226215563702086;  % kg
smiData.Solid(3).CoM = [1.6489056665980748 -0.16630841243393651 4.8005471667009605];  % mm
smiData.Solid(3).MoI = [0.012505437816752082 0.012045678943281296 0.0090116007810890388];  % kg*mm^2
smiData.Solid(3).PoI = [0 -0.0025244411463353463 0];  % kg*mm^2
smiData.Solid(3).color = [0.89411764705882346 0.89411764705882346 0.89411764705882346];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Part2^Assem1*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(2).Rz.Pos = 0.0;
smiData.RevoluteJoint(2).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -89.19808700052765;  % deg
smiData.RevoluteJoint(1).ID = "[Part2^Assem1-2:-:Part3^Assem1-1]";

smiData.RevoluteJoint(2).Rz.Pos = -173.13984612012348;  % deg
smiData.RevoluteJoint(2).ID = "[Part1^Assem1-1:-:Part2^Assem1-2]";

