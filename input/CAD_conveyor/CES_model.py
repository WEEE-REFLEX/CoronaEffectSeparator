# Chrono::Engine Python script from SolidWorks 
# Assembly: C:\Users\Alessandro\Dropbox\Cad_CES\Cad_ida_v2\CAD_conveyor\Assembly_Ida_aperto.SLDASM


import ChronoEngine_PYTHON_core as chrono 
import builtins 

shapes_dir = 'CES_model_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('Spazzola-1')
body_1.SetPos(chrono.ChVectorD(-0.0339264149957167,0.433220509447178,-0.0817847646755438))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(2.60413125593896)
body_1.SetInertiaXX(chrono.ChVectorD(0.0213811392305318,0.0213811392305318,0.00358414990389192))
body_1.SetInertiaXY(chrono.ChVectorD(-1.3656634769519e-35,1.59975608589952e-20,-4.47399006138431e-38))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.01787213456557e-18,1.29343073635046e-34,6.30201082054704e-19),chrono.ChQuaternionD(1,0,0,0)))

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
marker_1_1.SetName('Spazzola')
body_1.AddMarker(marker_1_1)
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.0339264149957167,0.433220509447178,-0.0817847646755438),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddCylinder(0.053,0.053,0.15,chrono.ChVectorD(0,0,0),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('conveyor-1')
body_2.SetPos(chrono.ChVectorD(-0.0627155647639022,0.0534995314229745,-0.0817847646755438))
body_2.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2.SetMass(3.83374780369416)
body_2.SetInertiaXX(chrono.ChVectorD(0.0293230020631821,0.0796603555733116,0.0514771405810813))
body_2.SetInertiaXY(chrono.ChVectorD(0.000231478123253108,-3.23796596160847e-18,3.2149801182765e-19))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0716585043775775,0.521137386832319,-6.62701667401117e-19),chrono.ChQuaternionD(1,0,0,0)))
body_2.SetBodyFixed(True)

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
marker_2_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.00849641067492329,0.587493980472233,-0.0817847646755438),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.995090127679177; mr[1,1]=-0.0989729144535993; mr[2,1]=0 
mr[0,2]=-0.0989729144535993; mr[1,2]=-0.995090127679177; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(0.15,0.193724352473189,0.005,chrono.ChVectorD(0.0781418660583536,0.550182642431129,0),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('Splitter-10')
body_3.SetPos(chrono.ChVectorD(0.324996410674918,0.273993980472249,-0.0817847646755438))
body_3.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3.SetMass(0.136861816137753)
body_3.SetInertiaXX(chrono.ChVectorD(0.00120251356875357,0.00102728888618058,0.000176832203169258))
body_3.SetInertiaXY(chrono.ChVectorD(-1.139974266205e-09,-1.39829005251977e-20,7.78994144023331e-20))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00150052115485541,-0.0159825459116232,6.09494858549617e-19),chrono.ChQuaternionD(1,0,0,0)))

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
marker_3_1.SetName('Splitter1')
body_3.AddMarker(marker_3_1)
marker_3_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.324996410674918,0.273993980472249,-0.0817847646755438),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.1515,0.0535,0.002,chrono.ChVectorD(0.002,6.93889390390723E-18,5.55111512312578E-17),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('drum-1')
body_4.SetPos(chrono.ChVectorD(-0.0927155647639022,0.0134995314229745,-0.0817847646755438))
body_4.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_4.SetMass(24.0151127638079)
body_4.SetInertiaXX(chrono.ChVectorD(0.332285545611457,0.332285545611457,0.308075522985403))
body_4.SetInertiaXY(chrono.ChVectorD(-1.01246906976491e-18,-9.51639207107421e-19,-1.32339629190176e-19))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.271211975438826,0.413994449049259,-1.66242648262286e-06),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_4.GetAssets().push_back(body_4_1_level) 

# Auxiliary marker (coordinate system feature)
marker_4_1 =chrono.ChMarker()
marker_4_1.SetName('centro_cilindro')
body_4.AddMarker(marker_4_1)
marker_4_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.178496410674923,0.427493980472233,-0.0817847646755438),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_4.GetCollisionModel().AddCylinder(0.16,0.16,0.15,chrono.ChVectorD(0.271211975438826,0.413994449049259,0),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('KWS-L min-1')
body_5.SetPos(chrono.ChVectorD(0.181496410674923,0.423393980472242,-0.0817847646755503))
body_5.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_5.SetMass(0.575578251261412)
body_5.SetInertiaXX(chrono.ChVectorD(0.229283207371073,0.138637511031281,0.320113118266443))
body_5.SetInertiaXY(chrono.ChVectorD(-0.0883080169952972,0.0439356443408866,0.0345485433857679))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.11937745010495,-0.309630192205084,0.0499780095445471),chrono.ChQuaternionD(1,0,0,0)))
body_5.SetBodyFixed(True)

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_5.GetAssets().push_back(body_5_1_level) 

# Auxiliary marker (coordinate system feature)
marker_5_1 =chrono.ChMarker()
marker_5_1.SetName('centro_nozzle')
body_5.AddMarker(marker_5_1)
marker_5_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.112244563580475,0.611594410584307,-0.0817847646755503),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_5_2 =chrono.ChMarker()
marker_5_2.SetName('centro_electrode')
body_5.AddMarker(marker_5_2)
marker_5_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.438304031810255,0.577493980472233,-0.0817847646755503),chrono.ChQuaternionD(1,0,0,0)))

# Collision shapes 
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-0.257282066836211; mr[1,0]=-0.966336348320028; mr[2,0]=0 
mr[0,1]=0.966336348320028; mr[1,1]=-0.257282066836211; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_5.GetCollisionModel().AddBox(0.111244909897976,0.01,0.165,chrono.ChVectorD(0.463991547662121,-0.621358743768626,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.257282066836212; mr[1,0]=-0.966336348320028; mr[2,0]=0 
mr[0,1]=0.966336348320028; mr[1,1]=0.257282066836212; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_5.GetCollisionModel().AddBox(0.111244909897975,0.00991722943912151,0.165,chrono.ChVectorD(0.287088436539427,-0.62133744838765,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.257282066836211; mr[1,0]=0.966336348320028; mr[2,0]=0 
mr[0,1]=0.966336348320028; mr[1,1]=-0.257282066836211; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_5.GetCollisionModel().AddBox(0.111244909897976,0.018292686227323,0.16500005,chrono.ChVectorD(0.455978023535446,-0.619225184316436,5.00000000014378E-08),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.252626364958053; mr[1,0]=-0.967563909893337; mr[2,0]=0 
mr[0,1]=0.967563909893337; mr[1,1]=-0.252626364958051; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_5.GetCollisionModel().AddBox(0.110302982718799,0.009073557274052,0.165,chrono.ChVectorD(0.201202979597005,-0.620296344571127,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1.79668909440449E-07; mr[1,0]=0; mr[2,0]=-0.999999999999984 
mr[0,1]=-1.09200594454053E-13; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=-0.999999999999984; mr[1,2]=1.09200594454051E-13; mr[2,2]=1.79668909440449E-07 
body_5.GetCollisionModel().AddBox(0.0110424966507172,0.250103825371303,0.448934434860075,chrono.ChVectorD(0.0344345849372048,-1.22700356432596,-0.179542665947878),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.092913431136299; mr[1,0]=-0.995674190844817; mr[2,0]=0 
mr[0,1]=0.995674190844817; mr[1,1]=0.0929134311362961; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_5.GetCollisionModel().AddBox(0.102920917614647,0.00880730074763987,0.165,chrono.ChVectorD(0.0461378510072952,-0.614333388045196,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1.79668910336172E-07; mr[1,0]=-1.09131972701093E-14; mr[2,0]=0.999999999999984 
mr[0,1]=1.09203455647929E-13; mr[1,1]=1; mr[2,1]=3.88032583520562E-16 
mr[0,2]=-0.999999999999984; mr[1,2]=1.0920345557821E-13; mr[2,2]=1.79668910336172E-07 
body_5.GetCollisionModel().AddBox(0.0101732150271489,0.250351436400839,0.448934434860075,chrono.ChVectorD(0.0344346606165717,-1.22725133002599,0.241673041703092),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.469231752670232; mr[1,0]=-0.883075060391823; mr[2,0]=0 
mr[0,1]=0.883075060391824; mr[1,1]=-0.46923175267023; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_5.GetCollisionModel().AddBox(0.121733706251767,0.00901290521459452,0.165,chrono.ChVectorD(-0.0943376359362894,-0.62333298893951,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.469231752670225; mr[1,0]=0.883075060391827; mr[2,0]=0 
mr[0,1]=-0.883075060391827; mr[1,1]=-0.469231752670225; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_5.GetCollisionModel().AddBox(0.121733706251767,0.00876657438055399,0.165,chrono.ChVectorD(-0.324444835447566,-0.623217402690516,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1.79668909389275E-07; mr[1,0]=0; mr[2,0]=0.999999999999984 
mr[0,1]=1.09018457105282E-13; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-0.999999999999984; mr[1,2]=1.0901845710528E-13; mr[2,2]=1.79668909389275E-07 
body_5.GetCollisionModel().AddBox(0.221215709664422,0.0137481406638926,0.448934434860076,chrono.ChVectorD(0.0344346226988081,-1.12355300540107,0.0306305470658227),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(0.0246194369069635,0.0246194369069635,0.16,chrono.ChVectorD(0.244352690753348,-0.517959057661396,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(0.0124935472670776,0.0124935472670776,0.16,chrono.ChVectorD(0.492612868005681,-0.513858743768626,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(0.00876657438055399,0.00876657438055399,0.16,chrono.ChVectorD(-0.381566155791125,-0.515717402690516,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(0.0458692620536463,0.0458692620536463,0.16,chrono.ChVectorD(0,-0.517959057661396,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-0.928115067405019; mr[2,0]=0.372293461741913 
mr[0,1]=0; mr[1,1]=0.372293461741912; mr[2,1]=0.928115067405019 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddBox(0.115826155371625,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.624455203991318,-0.106974099367162),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(0.0147645993377839,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.516955203991318,-0.150095419710721),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-0.928115067405019; mr[2,0]=-0.372293461741913 
mr[0,1]=0; mr[1,1]=0.372293461741912; mr[2,1]=-0.928115067405019 
mr[0,2]=1; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddBox(0.115826155371625,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.624455203991318,0.106974099367162),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(0.0147645993377839,0.0147645993377839,0.447000000000001,chrono.ChVectorD(0.0544999999999997,-0.516955203991318,0.150095419710721),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.999999999999984; mr[1,0]=-1.08039712596561E-13; mr[2,0]=-1.79668909431761E-07 
mr[0,1]=0; mr[1,1]=0.999999999999815; mr[2,1]=-6.07857698725874E-07 
mr[0,2]=1.79668909431793E-07; mr[1,2]=6.07857698725865E-07; mr[2,2]=0.999999999999799 
body_5.GetCollisionModel().AddBox(0.0154140963254538,0.25013019465113,0.2035,chrono.ChVectorD(0.467954961389621,-1.22702981403608,0.0314999008291814),mr)
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.724206595963648,-0.0075560261331982))
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.509206595963647,-0.0937986668203162))
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.522451967790014,-0.163798666820316))
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.737451967790015,-0.0775560261331982))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.724206595963648,-0.0075560261331982))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.509206595963647,-0.0937986668203162))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.522451967790014,-0.163798666820316))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.737451967790015,-0.0775560261331982))
body_5.GetCollisionModel().AddConvexHull(pt_vect)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.99999999999982; mr[1,0]=-5.71950500313773E-07; mr[2,0]=1.79669257143281E-07 
mr[0,1]=5.71950606677362E-07; mr[1,1]=-0.999999999999652; mr[2,1]=6.07857595951248E-07 
mr[0,2]=1.79668909478763E-07; mr[1,2]=6.0785769871308E-07; mr[2,2]=0.999999999999799 
body_5.GetCollisionModel().AddBox(0.0197636915985527,0.250242141495163,0.2035,chrono.ChVectorD(0.209277316591803,-1.22714190001486,0.0314999473736441),mr)
pt_vect = chrono.vector_ChVectorD()
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.724206595963648,0.0075560261331982))
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.509206595963647,0.0937986668203162))
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.522451967790014,0.163798666820316))
pt_vect.push_back(chrono.ChVectorD(-0.3845,-0.737451967790015,0.0775560261331982))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.724206595963648,0.0075560261331982))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.509206595963647,0.0937986668203162))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.522451967790014,0.163798666820316))
pt_vect.push_back(chrono.ChVectorD(0.5015001,-0.737451967790015,0.0775560261331982))
body_5.GetCollisionModel().AddConvexHull(pt_vect)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.99999999999982; mr[1,0]=-5.71950496936566E-07; mr[2,0]=1.79669257288503E-07 
mr[0,1]=5.71950606982801E-07; mr[1,1]=-0.999999999999652; mr[2,1]=6.07857595951683E-07 
mr[0,2]=1.79668909623986E-07; mr[1,2]=6.07857698713514E-07; mr[2,2]=0.999999999999799 
body_5.GetCollisionModel().AddBox(0.0292554789734579,0.249805368165055,0.2035,chrono.ChVectorD(-0.0617418466516264,-1.2267052816943,0.0314999958019598),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.999999999999984; mr[1,0]=1.08917894408591E-13; mr[2,0]=1.79668910122167E-07 
mr[0,1]=0; mr[1,1]=-0.999999999999815; mr[2,1]=6.07857698766414E-07 
mr[0,2]=1.796689101222E-07; mr[1,2]=6.07857698766404E-07; mr[2,2]=0.999999999999799 
body_5.GetCollisionModel().AddBox(0.0132511736464386,0.254046190843515,0.2035,chrono.ChVectorD(-0.401248638358623,-1.23094629097478,0.031500059378705),mr)
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(True)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('Splitter2-1')
body_6.SetPos(chrono.ChVectorD(0.416156012458144,0.286549383384347,-0.0817847646755439))
body_6.SetRot(chrono.ChQuaternionD(0.972369920397677,1.04672748231844e-16,-2.51297035050063e-17,0.233445363855904))
body_6.SetMass(0.169782296961291)
body_6.SetInertiaXX(chrono.ChVectorD(0.00159993103963307,0.00135874722183981,0.000411943807053511))
body_6.SetInertiaXY(chrono.ChVectorD(-0.00016598313102536,-1.37223550733389e-19,-2.82657551660551e-19))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00150058309966217,-0.0154834802535394,-2.77148128985996e-17),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_6.GetAssets().push_back(body_6_1_level) 

# Auxiliary marker (coordinate system feature)
marker_6_1 =chrono.ChMarker()
marker_6_1.SetName('Splitter2')
body_6.AddMarker(marker_6_1)
marker_6_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.416156012458144,0.286549383384347,-0.0817847646755439),chrono.ChQuaternionD(0.972369920397677,1.04672748231844E-16,-2.51297035050063E-17,0.233445363855904)))

# Collision shapes 
body_6.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_6.GetCollisionModel().AddBox(0.15,0.075,0.002,chrono.ChVectorD(0.002,1.38777878078145E-17,0),mr)
body_6.GetCollisionModel().BuildModel()
body_6.SetCollide(True)

exported_items.append(body_6)




# Mate constraint: Coincident1 [MateCoincident]
#   Entity 0: C::E name: body_5 , SW name: KWS-L min-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:4


# Mate constraint: Distance1 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_5 , SW name: KWS-L min-1 ,  SW ref.type:2

link_1 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.191503589325077,0.587493980472233,0.0682152353244562)
cB = chrono.ChVectorD(0.154996410674922,-0.0875060195277686,0.0782152353244314)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(-3.54919571694725e-15,-1,-5.5180472374077e-15)
link_1.Initialize(body_2,body_5,False,cA,cB,dB)
link_1.SetDistance(-0.675)
link_1.SetName("Distance1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.191503589325077,0.587493980472233,0.0682152353244562)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(0.154996410674922,-0.0875060195277686,0.0782152353244314)
dB = chrono.ChVectorD(-3.54919571694725e-15,-1,-5.5180472374077e-15)
link_2.SetFlipped(True)
link_2.Initialize(body_2,body_5,False,cA,cB,dA,dB)
link_2.SetName("Distance1")
exported_items.append(link_2)


# Mate constraint: Distance2 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_5 , SW name: KWS-L min-1 ,  SW ref.type:2

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.208496410674923,0.587493980472233,0.0682152353244562)
cB = chrono.ChVectorD(-0.211503589325077,0.901493980472241,0.147215235324446)
dA = chrono.ChVectorD(1,0,0)
dB = chrono.ChVectorD(-1,-5.20742506859829e-17,5.20742506859829e-17)
link_3.Initialize(body_2,body_5,False,cA,cB,dB)
link_3.SetDistance(-0.42)
link_3.SetName("Distance2")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.208496410674923,0.587493980472233,0.0682152353244562)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(-0.211503589325077,0.901493980472241,0.147215235324446)
dB = chrono.ChVectorD(-1,-5.20742506859829e-17,5.20742506859829e-17)
link_4.SetFlipped(True)
link_4.Initialize(body_2,body_5,False,cA,cB,dA,dB)
link_4.SetName("Distance2")
exported_items.append(link_4)


# Mate constraint: Perpendicular1 [MatePerpendicular]
#   Entity 0: C::E name: body_5 , SW name: KWS-L min-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:24


# Mate constraint: Distance4 [MateDistanceDim]
#   Entity 0: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:25
#   Entity 1: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2


# Mate constraint: Distance5 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:2
#   Entity 1: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:25


# Mate constraint: Coincident4 [MateCoincident]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:4


# Mate constraint: Coincidente3 [MateCoincident]
#   Entity 0: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_3 , SW name: Splitter-10 ,  SW ref.type:4


# Mate constraint: Angolo1 [MatePlanarAngleDim]
#   Entity 0: C::E name: body_3 , SW name: Splitter-10 ,  SW ref.type:4
#   Entity 1: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:4


# Mate constraint: Coincidente6 [MateCoincident]
#   Entity 0: C::E name: body_3 , SW name: Splitter-10 ,  SW ref.type:4
#   Entity 1: C::E name: body_6 , SW name: Splitter2-1 ,  SW ref.type:4


# Mate constraint: Coincidente8 [MateCoincident]
#   Entity 0: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_1 , SW name: Spazzola-1 ,  SW ref.type:4


# Mate constraint: Distanza2 [MateDistanceDim]
#   Entity 0: C::E name: body_4 , SW name: drum-1 ,  SW ref.type:5
#   Entity 1: C::E name: body_1 , SW name: Spazzola-1 ,  SW ref.type:0


# Mate constraint: Distanza3 [MateDistanceDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:0
#   Entity 1: C::E name: body_1 , SW name: Spazzola-1 ,  SW ref.type:0


# Mate constraint: Angolo3 [MatePlanarAngleDim]
#   Entity 0: C::E name: body_2 , SW name: conveyor-1 ,  SW ref.type:4
#   Entity 1: C::E name: body_6 , SW name: Splitter2-1 ,  SW ref.type:4


# Mate constraint: Concentrico1 [MateConcentric]
#   Entity 0: C::E name: body_5 , SW name: KWS-L min-1 ,  SW ref.type:1
#   Entity 1: C::E name: body_6 , SW name: Splitter2-1 ,  SW ref.type:1

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.451496410674919,0.220493980472247,0.190715235324296)
dA = chrono.ChVectorD(0,-2.15294089288643e-16,1)
cB = chrono.ChVectorD(0.451496410674919,0.220493980472248,0.068215235324456)
dB = chrono.ChVectorD(3.20474742746036e-31,-2.15294089288644e-16,1)
link_5.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_5.SetName("Concentrico1")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.451496410674919,0.220493980472247,0.190715235324296)
cB = chrono.ChVectorD(0.451496410674919,0.220493980472248,0.068215235324456)
dA = chrono.ChVectorD(0,-2.15294089288643e-16,1)
dB = chrono.ChVectorD(3.20474742746036e-31,-2.15294089288644e-16,1)
link_6.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_6.SetName("Concentrico1")
exported_items.append(link_6)


# Mate constraint: Concentrico2 [MateConcentric]
#   Entity 0: C::E name: body_5 , SW name: KWS-L min-1 ,  SW ref.type:1
#   Entity 1: C::E name: body_3 , SW name: Splitter-10 ,  SW ref.type:1

link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.326496410674919,0.220493980472247,0.191715235324296)
dA = chrono.ChVectorD(0,2.15294089288643e-16,-1)
cB = chrono.ChVectorD(0.326496410674918,0.220493980472249,0.0682152353244562)
dB = chrono.ChVectorD(0,0,1)
link_7.SetFlipped(True)
link_7.Initialize(body_5,body_3,False,cA,cB,dA,dB)
link_7.SetName("Concentrico2")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateGeneric()
link_8.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.326496410674919,0.220493980472247,0.191715235324296)
cB = chrono.ChVectorD(0.326496410674918,0.220493980472249,0.0682152353244562)
dA = chrono.ChVectorD(0,2.15294089288643e-16,-1)
dB = chrono.ChVectorD(0,0,1)
link_8.Initialize(body_5,body_3,False,cA,cB,dA,dB)
link_8.SetName("Concentrico2")
exported_items.append(link_8)

