# Chrono::Engine Python script from SolidWorks
# Assembly: C:\Users\tasora\Desktop\conveyor\CAD_conveyor\Assembly_Ida_aperto.SLDASM


import ChronoEngine_PYTHON_core as chrono
import builtins

shapes_dir = 'conveyor_Ida_shapes/'

if hasattr(builtins, 'exported_system_relpath'):
    shapes_dir = builtins.exported_system_relpath + shapes_dir

exported_items = []

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('KWS-L min-1')
body_1.SetPos(chrono.ChVectorD(0.181496410674923,0.423393980472242,-0.0817847646755503))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(0.470498552470899)
body_1.SetInertiaXX(chrono.ChVectorD(0.202294062438838,0.139268704121435,0.295143554677343))
body_1.SetInertiaXY(chrono.ChVectorD(-0.107478714409304,0.0444121518889661,0.0372568393227256))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.13594225825414,-0.21548182408939,0.0611399440792479),chrono.ChQuaternionD(1,0,0,0)))
body_1.SetBodyFixed(True)

# Visualization shape
body_1_1_shape = chrono.ChObjShapeFile()
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj')
body_1_1_level = chrono.ChAssetLevel()
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1_1_level.GetAssets().push_back(body_1_1_shape)
body_1.GetAssets().push_back(body_1_1_level)

# Auxiliary marker (coordinate system feature)
marker_1_1 =chrono.ChMarker()
marker_1_1.SetName('centro_nozzle')
body_1.AddMarker(marker_1_1)
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.0172445635804752,0.553594410584307,-0.0817847646755503),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_1_2 =chrono.ChMarker()
marker_1_2.SetName('centro_electrode')
body_1.AddMarker(marker_1_2)
marker_1_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.511605301999477,0.573493980472231,-0.0817847646755503),chrono.ChQuaternionD(1,0,0,0)))

# Collision shape(s)
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1.79668910336172E-07; mr[1,0]=-1.09131972701093E-14; mr[2,0]=0.999999999999984
mr[0,1]=1.09203455647929E-13; mr[1,1]=1; mr[2,1]=3.88032583520562E-16
mr[0,2]=-0.999999999999984; mr[1,2]=1.0920345557821E-13; mr[2,2]=1.79668910336172E-07
body_1.GetCollisionModel().AddBox(0.0101732150271489,0.250351436400839,0.448934434860075,chrono.ChVectorD(0.0344346606165717,-1.22725133002599,0.241673041703092),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.469231752670232; mr[1,0]=-0.883075060391823; mr[2,0]=0
mr[0,1]=0.883075060391824; mr[1,1]=-0.46923175267023; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1
body_1.GetCollisionModel().AddBox(0.121733706251767,0.00901290521459452,0.165,chrono.ChVectorD(-0.0943376359362894,-0.62333298893951,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.469231752670225; mr[1,0]=0.883075060391827; mr[2,0]=0
mr[0,1]=-0.883075060391827; mr[1,1]=-0.469231752670225; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1
body_1.GetCollisionModel().AddBox(0.121733706251767,0.00876657438055399,0.165,chrono.ChVectorD(-0.324444835447566,-0.623217402690516,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1.79668909389275E-07; mr[1,0]=0; mr[2,0]=0.999999999999984
mr[0,1]=1.09018457105282E-13; mr[1,1]=1; mr[2,1]=0
mr[0,2]=-0.999999999999984; mr[1,2]=1.0901845710528E-13; mr[2,2]=1.79668909389275E-07
body_1.GetCollisionModel().AddBox(0.221215709664422,0.0137481406638926,0.448934434860076,chrono.ChVectorD(0.0344346226988081,-1.12355300540107,0.0306305470658227),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.0246194369069635,0.0246194369069635,0.16,chrono.ChVectorD(0.244352690753348,-0.517959057661396,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.0124935472670776,0.0124935472670776,0.16,chrono.ChVectorD(0.492612868005681,-0.513858743768626,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.00876657438055399,0.00876657438055399,0.16,chrono.ChVectorD(-0.381566155791125,-0.515717402690516,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.0458692620536463,0.0458692620536463,0.16,chrono.ChVectorD(0,-0.517959057661396,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-0.928115067405019; mr[2,0]=0.372293461741913
mr[0,1]=0; mr[1,1]=0.372293461741912; mr[2,1]=0.928115067405019
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.115826155371625,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.624455203991318,-0.106974099367162),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=0
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.0147645993377839,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.516955203991318,-0.150095419710721),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-0.928115067405019; mr[2,0]=-0.372293461741913
mr[0,1]=0; mr[1,1]=0.372293461741912; mr[2,1]=-0.928115067405019
mr[0,2]=1; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddBox(0.115826155371625,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.624455203991318,0.106974099367162),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=0
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=0
body_1.GetCollisionModel().AddCylinder(0.0147645993377839,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.516955203991318,0.150095419710721),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.999999999999984; mr[1,0]=-1.08039712596561E-13; mr[2,0]=-1.79668909431761E-07
mr[0,1]=0; mr[1,1]=0.999999999999815; mr[2,1]=-6.07857698725874E-07
mr[0,2]=1.79668909431793E-07; mr[1,2]=6.07857698725865E-07; mr[2,2]=0.999999999999799
body_1.GetCollisionModel().AddBox(0.0154140963254538,0.25013019465113,0.2035,chrono.ChVectorD(0.467954961389621,-1.22702981403608,0.0314999008291814),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.99999999999982; mr[1,0]=-5.71950500313773E-07; mr[2,0]=1.79669257143281E-07
mr[0,1]=5.71950606677362E-07; mr[1,1]=-0.999999999999652; mr[2,1]=6.07857595951248E-07
mr[0,2]=1.79668909478763E-07; mr[1,2]=6.0785769871308E-07; mr[2,2]=0.999999999999799
body_1.GetCollisionModel().AddBox(0.0197636915985527,0.250242141495163,0.2035,chrono.ChVectorD(0.209277316591803,-1.22714190001486,0.0314999473736441),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.99999999999982; mr[1,0]=-5.71950496936566E-07; mr[2,0]=1.79669257288503E-07
mr[0,1]=5.71950606982801E-07; mr[1,1]=-0.999999999999652; mr[2,1]=6.07857595951683E-07
mr[0,2]=1.79668909623986E-07; mr[1,2]=6.07857698713514E-07; mr[2,2]=0.999999999999799
body_1.GetCollisionModel().AddBox(0.0292554789734579,0.249805368165055,0.2035,chrono.ChVectorD(-0.0617418466516264,-1.2267052816943,0.0314999958019598),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.999999999999984; mr[1,0]=1.08917894408591E-13; mr[2,0]=1.79668910122167E-07
mr[0,1]=0; mr[1,1]=-0.999999999999815; mr[2,1]=6.07857698766414E-07
mr[0,2]=1.796689101222E-07; mr[1,2]=6.07857698766404E-07; mr[2,2]=0.999999999999799
body_1.GetCollisionModel().AddBox(0.0132511736464386,0.254046190843515,0.2035,chrono.ChVectorD(-0.401248638358623,-1.23094629097478,0.031500059378705),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.257282066836211; mr[1,0]=-0.966336348320028; mr[2,0]=0
mr[0,1]=0.966336348320028; mr[1,1]=-0.257282066836211; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1
body_1.GetCollisionModel().AddBox(0.111244909897976,0.01,0.165,chrono.ChVectorD(0.463991547662121,-0.621358743768626,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.257282066836212; mr[1,0]=-0.966336348320028; mr[2,0]=0
mr[0,1]=0.966336348320028; mr[1,1]=0.257282066836212; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1
body_1.GetCollisionModel().AddBox(0.111244909897975,0.00991722943912151,0.165,chrono.ChVectorD(0.287088436539427,-0.62133744838765,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.252626364958053; mr[1,0]=-0.967563909893337; mr[2,0]=0
mr[0,1]=0.967563909893337; mr[1,1]=-0.252626364958051; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1
body_1.GetCollisionModel().AddBox(0.110302982718799,0.009073557274052,0.165,chrono.ChVectorD(0.201202979597005,-0.620296344571127,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1.79668909440449E-07; mr[1,0]=0; mr[2,0]=-0.999999999999984
mr[0,1]=-1.09200594454053E-13; mr[1,1]=-1; mr[2,1]=0
mr[0,2]=-0.999999999999984; mr[1,2]=1.09200594454051E-13; mr[2,2]=1.79668909440449E-07
body_1.GetCollisionModel().AddBox(0.0110424966507172,0.250103825371303,0.448934434860075,chrono.ChVectorD(0.0344345849372048,-1.22700356432596,-0.179542665947878),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.092913431136299; mr[1,0]=-0.995674190844817; mr[2,0]=0
mr[0,1]=0.995674190844817; mr[1,1]=0.0929134311362961; mr[2,1]=0
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1
body_1.GetCollisionModel().AddBox(0.102920917614647,0.00880730074763987,0.165,chrono.ChVectorD(0.0461378510072952,-0.614333388045196,0),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('conveyor-1')
body_2.SetPos(chrono.ChVectorD(-0.0627155647639021,-0.0215004685770272,-0.0817847646755531))
body_2.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2.SetMass(3.6)
body_2.SetInertiaXX(chrono.ChVectorD(0.02727,0.0750000000000001,0.0482700000000001))
body_2.SetInertiaXY(chrono.ChVectorD(-1.73472347597681e-17,-2.89120579329468e-18,2.16840434497101e-19))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0712119754388255,0.518994449049259,1.50583635067428e-19),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_2_1_shape = chrono.ChObjShapeFile()
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj')
body_2_1_level = chrono.ChAssetLevel()
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2_1_level.GetAssets().push_back(body_2_1_shape)
body_2.GetAssets().push_back(body_2_1_level)

# Auxiliary marker (coordinate system feature)
marker_2_1 =chrono.ChMarker()
marker_2_1.SetName('centro_nastro')
body_2.AddMarker(marker_2_1)
marker_2_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.00849641067492332,0.512493980472232,-0.0817847646755531),chrono.ChQuaternionD(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('drum-1')
body_3.SetPos(chrono.ChVectorD(-0.0627155647639021,-0.0155004685770276,-0.0817847646755503))
body_3.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3.SetMass(12.1230557328982)
body_3.SetInertiaXX(chrono.ChVectorD(0.128348659523282,0.128348659523282,0.0785826064029088))
body_3.SetInertiaXY(chrono.ChVectorD(-1.74700545371199e-18,-7.45150209118528e-19,-4.7640576724812e-19))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.271211975438826,0.413994449049259,-3.29317626854362e-06),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape
body_3_1_shape = chrono.ChObjShapeFile()
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj')
body_3_1_level = chrono.ChAssetLevel()
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0))
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3_1_level.GetAssets().push_back(body_3_1_shape)
body_3.GetAssets().push_back(body_3_1_level)

# Auxiliary marker (coordinate system feature)
marker_3_1 =chrono.ChMarker()
marker_3_1.SetName('centro_cilindro')
body_3.AddMarker(marker_3_1)
marker_3_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.208496410674923,0.398493980472231,-0.0817847646755503),chrono.ChQuaternionD(1,0,0,0)))

# Collision shape(s)
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0
body_3.GetCollisionModel().AddCylinder(0.114,0.114,0.15,chrono.ChVectorD(0.271211975438826,0.413994449049259,0),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)




# Mate constraint: Coincident1 [MateCoincident]
#   Entity 0: C::E name: body_1 , SW name: KWS-L min-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:4


# Mate constraint: Distance1 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_1 , SW name: KWS-L min-1 ,  SW ref.type:2

link_1 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.191503589325077,0.512493980472232,0.0682152353244469)
cB = chrono.ChVectorD(0.154996410674922,-0.0875060195277686,0.0782152353244314)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(-3.54919571694725e-15,-1,-5.5180472374077e-15)
link_1.Initialize(body_2,body_1,False,cA,cB,dB)
link_1.SetDistance(-0.6)
link_1.SetName("Distance1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.191503589325077,0.512493980472232,0.0682152353244469)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.154996410674922,-0.0875060195277686,0.0782152353244314)
dB = chrono.ChVectorD(-3.54919571694725e-15,-1,-5.5180472374077e-15)
link_2.SetFlipped(True)
link_2.Initialize(body_2,body_1,False,cA,cB,dA,dB)
link_2.SetName("Distance1")
exported_items.append(link_2)


# Mate constraint: Distance2 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_1 , SW name: KWS-L min-1 ,  SW ref.type:2

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.208496410674923,0.512493980472232,0.0682152353244469)
cB = chrono.ChVectorD(-0.211503589325077,0.901493980472241,0.147215235324446)
dA = chrono.ChVectorD(1,0,0)
dB = chrono.ChVectorD(-1,-5.20742506859829e-17,5.20742506859829e-17)
link_3.Initialize(body_2,body_1,False,cA,cB,dB)
link_3.SetDistance(-0.42)
link_3.SetName("Distance2")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.208496410674923,0.512493980472232,0.0682152353244469)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(-0.211503589325077,0.901493980472241,0.147215235324446)
dB = chrono.ChVectorD(-1,-5.20742506859829e-17,5.20742506859829e-17)
link_4.SetFlipped(True)
link_4.Initialize(body_2,body_1,False,cA,cB,dA,dB)
link_4.SetName("Distance2")
exported_items.append(link_4)


# Mate constraint: Perpendicular1 [MatePerpendicular]
#   Entity 0: C::E name: body_1 , SW name: KWS-L min-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_3 , SW name: drum-1 ,  SW ref.type:24


# Mate constraint: Distance4 [MateDistanceDim]
#   Entity 0: C::E name: body_3 , SW name: drum-1 ,  SW ref.type:25
#   Entity 1: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2


# Mate constraint: Distance5 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_3 , SW name: drum-1 ,  SW ref.type:25


# Mate constraint: Coincident4 [MateCoincident]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_3 , SW name: drum-1 ,  SW ref.type:4

