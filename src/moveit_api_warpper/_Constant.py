from geometry_msgs.msg import PoseStamped, Pose

memoeryChip_camera_detection= [-0.3131878674030304, 0.49928998947143555, 1.9762904644012451, 0.9798975586891174, 1.3386096954345703, -1.6718480587005615]

cpu_camera_detection= [1.0064616203308105, 0.5986682176589966, 0.8863067030906677, 3.1495535373687744, -1.6331021785736084, -1.3761985301971436]

home = [0.4102410078048706, 0.5366256237030029, 1.2140278816223145, 2.9551632404327393, -1.4722083806991577, -3.4526872634887695]

memoeryChip_pickJoint_1 = [0.7791017889976501, 0.7673542499542236, 1.293067455291748, 3.036687135696411, -1.1712076663970947, -3.0632681846618652]

memoeryChip_pickJoint_2 = [0.7791037559509277, 0.7785102128982544, 1.2986255884170532, 3.0359303951263428, -1.154585599899292, -3.061354875564575]

anomalyJoint = [0.4934387803077698, 0.6360915899276733, 1.4535309076309204, 3.1096231937408447, -1.0606051683425903, -3.4014620780944824]

preNomalJoint = [0.4898624122142792, 0.6311354637145996, 1.4573891162872314, 3.1096091270446777, -1.061598300933838, -3.405020236968994]
nomalJoint = [0.489859402179718, 0.6436320543289185, 1.4648383855819702, 3.1092405319213867, -1.041667103767395, -3.4042787551879883]
prepick_offset = 0.17
preinsert_offset = 0.17

memoeryChip_pickPose = Pose()
memoeryChip_pickPose.position.x = 0.477944139903
memoeryChip_pickPose.position.y = 0.542397797823
memoeryChip_pickPose.position.z = 0.473
memoeryChip_pickPose.orientation.x = 0.366112842563
memoeryChip_pickPose.orientation.y = -0.925109260935
memoeryChip_pickPose.orientation.z = 0.094844408107
memoeryChip_pickPose.orientation.w = 0.0337458159515

memoeryChip_insertPose = Pose()
memoeryChip_insertPose.position.x = 0.568462689829
memoeryChip_insertPose.position.y = 0.357238472434
memoeryChip_insertPose.position.z = 0.499778886991
memoeryChip_insertPose.orientation.x = 0.369692164759
memoeryChip_insertPose.orientation.y = -0.929043458791
memoeryChip_insertPose.orientation.z = 0.0136112936061
memoeryChip_insertPose.orientation.w = 0.00454837112943

memoeryChip_anomalyPose = Pose()
memoeryChip_anomalyPose.position.x = 0.608794578804
memoeryChip_anomalyPose.position.y = 0.392936547604
memoeryChip_anomalyPose.position.z = 0.503982321713
memoeryChip_anomalyPose.orientation.x = -0.367271091496
memoeryChip_anomalyPose.orientation.y = -0.930108039925
memoeryChip_anomalyPose.orientation.z = 0.00229926534125
memoeryChip_anomalyPose.orientation.w = 0.00238595849421


cpu_pickPose = Pose()
cpu_pickPose.position.x = 0.389964938732
cpu_pickPose.position.y = 0.652318908239
cpu_pickPose.position.z = 0.479
cpu_pickPose.orientation.x = 0.927640427381
cpu_pickPose.orientation.y = -0.372936390899
cpu_pickPose.orientation.z = 0.01954549467
cpu_pickPose.orientation.w = 0.00443390005975

cpu_insertPose = Pose()
cpu_insertPose.position.x = 0.623848058274
cpu_insertPose.position.y = 0.281956277387
cpu_insertPose.position.z = 0.499
cpu_insertPose.orientation.x = -0.370098384544
cpu_insertPose.orientation.y = -0.928907983007
cpu_insertPose.orientation.z = -0.0124659623909
cpu_insertPose.orientation.w = 0.00132085055592

