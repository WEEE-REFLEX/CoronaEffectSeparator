# Chrono::Engine Python script from SolidWorks 
# Assembly: C:\tasora\code\dynamics\papers\contact_rolling\plots\conveyor\assembly_conveyor.SLDASM


import ChronoEngine_PYTHON_core as chrono 
import builtins

shapes_dir = 'cad_conveyor_shapes/' 

if hasattr(builtins, 'exported_system_relpath'):
    shapes_dir = builtins.exported_system_relpath + shapes_dir
	
exported_items = [] 

body_0= chrono.ChBodyAuxRefShared()
body_0.SetName('ground')
body_0.SetBodyFixed(1)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRefShared()
body_1.SetName('conveyor^assembly_conveyor-1')
body_1.SetPos(chrono.ChVectorD(-1.5,1,0))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(62.5596464664722)
body_1.SetInertiaXX(chrono.ChVectorD(2.4969698065779,43.2055331519538,41.4723890032672))
body_1.SetInertiaXY(chrono.ChVectorD(-0.00109264755200904,4.73037407855426e-16,-5.11743425413158e-17))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000365893287965613,-0.0560030560102842,-3.54932640234839e-18),chrono.ChQuaternionD(1,0,0,0)))
body_1.SetBodyFixed(1)

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFileShared() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevelShared() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Auxiliary marker (coordinate system feature)
marker_1_1 =chrono.ChMarkerShared()
marker_1_1.SetName('Sistema di coordinate1')
body_1.AddMarker(marker_1_1)
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-1.5,1,0),chrono.ChQuaternionD(1,0,0,0)))

# Collision shape(s) 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddBox(0.03,0.0125,1.18,chrono.ChVectorD(0,0.03,0.2075),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=-1 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_1.GetCollisionModel().AddBox(0.03,0.0125,1.18,chrono.ChVectorD(0,0.03,-0.2075),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(1)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRefShared()
body_2.SetName('floor_box^assembly_conveyor-1')
body_2.SetPos(chrono.ChVectorD(0,0,0))
body_2.SetRot(chrono.ChQuaternionD(0,0,0.707106781186548,0.707106781186547))
body_2.SetMass(105.395469141052)
body_2.SetInertiaXX(chrono.ChVectorD(44.246695627893,87.9731845142143,44.2466956278934))
body_2.SetInertiaXY(chrono.ChVectorD(1.47353510596567e-14,-2.87373850207165e-14,-1.03112079646348e-14))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.49882282258257e-15,1.44127102887666e-15,0.0312371016149949),chrono.ChQuaternionD(1,0,0,0)))
body_2.SetBodyFixed(1)

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFileShared() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevelShared() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_2.GetAssets().push_back(body_2_1_level) 

# Collision shape(s) 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(1.02,1.02,0.05,chrono.ChVectorD(2.22044604925031E-16,-2.22044604925031E-16,-0.03),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(1.02,0.0100000000000001,0.1,chrono.ChVectorD(2.22044604925031E-16,1.01,0.1),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-3.483052626275E-15; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(1.02,0.0100000000000001,0.1,chrono.ChVectorD(-1.01,-3.33066907387547E-15,0.1),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(1.02,0.0100000000000001,0.1,chrono.ChVectorD(-2.22044604925031E-16,-1.01,0.1),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1.0449157878825E-14; mr[1,0]=-1; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=-1.11022302462515E-14; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_2.GetCollisionModel().AddBox(1.02,0.0100000000000001,0.1,chrono.ChVectorD(1.01,1.0325074129014E-14,0.1),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(1)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRefShared()
body_3.SetName('hopper^assembly_conveyor-1')
body_3.SetPos(chrono.ChVectorD(-2,1.02,0))
body_3.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_3.SetMass(15.3307320232533)
body_3.SetInertiaXX(chrono.ChVectorD(1.32219257974901,1.84835706903569,1.37402649799206))
body_3.SetInertiaXY(chrono.ChVectorD(-0.0305669736272472,9.02786496845205e-06,-3.03707665359274e-07))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.00791467065350081,0.325786014580203,-2.21504614754374e-06),chrono.ChQuaternionD(1,0,0,0)))
body_3.SetBodyFixed(1)

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFileShared() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevelShared() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Collision shape(s) 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=5.33221930687627E-16; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=1; mr[1,2]=-5.33221930687627E-16; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.36,0.0520525770195464,0.01,chrono.ChVectorD(0.36,0.547947422980454,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-0.919145030018058; mr[2,1]=-0.393919298579167 
mr[0,2]=0; mr[1,2]=-0.393919298579167; mr[2,2]=0.919145030018058 
body_3.GetCollisionModel().AddBox(0.35456016206724,0.190394327646598,0.01,chrono.ChVectorD(-0.00543983793276009,0.319008229994662,0.284191450300181),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0.393919298579168; mr[1,1]=-0.919145030018058; mr[2,1]=0 
mr[0,2]=-0.919145030018058; mr[1,2]=-0.393919298579168; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.355602280759736,0.190394327646598,0.01,chrono.ChVectorD(-0.284191450300181,0.321060807014208,-0.00560228075973579),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_3.GetCollisionModel().AddBox(0.36,0.0739737114902269,0.01,chrono.ChVectorD(1.11022302462516E-16,0.0739737114902269,0.21),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.20043983793276,0.075,0.01,chrono.ChVectorD(-0.21,0.075,-0.000439837932759923),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_3.GetCollisionModel().AddBox(0.36,0.0739737114902268,0.01,chrono.ChVectorD(1.11022302462516E-16,0.0739737114902268,-0.21),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_3.GetCollisionModel().AddBox(0.36,0.051026288509773,0.01,chrono.ChVectorD(1.11022302462516E-16,0.548973711490227,-0.36),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=-0.919145030018058; mr[2,1]=0.393919298579168 
mr[0,2]=0; mr[1,2]=-0.393919298579168; mr[2,2]=-0.919145030018058 
body_3.GetCollisionModel().AddBox(0.35456016206724,0.190394327646598,0.01,chrono.ChVectorD(0.00543983793276009,0.319008229994662,-0.284191450300181),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=-5.33221930687628E-16; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=-5.33221930687628E-16; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.36,0.0520525770195463,0.01,chrono.ChVectorD(-0.36,0.547947422980453,0),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_3.GetCollisionModel().AddBox(0.36,0.0510262885097731,0.01,chrono.ChVectorD(0,0.548973711490227,0.36),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=1 
mr[0,1]=-0.393919298579168; mr[1,1]=-0.919145030018058; mr[2,1]=0 
mr[0,2]=0.919145030018058; mr[1,2]=-0.393919298579168; mr[2,2]=0 
body_3.GetCollisionModel().AddBox(0.35,0.190394327646598,0.01,chrono.ChVectorD(0.284191450300181,0.321060807014208,5.55111512312578E-17),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(1)

exported_items.append(body_3)



