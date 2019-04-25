from geometry_msgs.msg import PoseStamped, Pose

memoeryChip_camera_detection= [-0.3131878674030304, 0.49928998947143555, 1.9762904644012451, 0.9798975586891174, 1.3386096954345703, -1.6718480587005615]

cpu_camera_detection= [0.7852657437324524, 0.6326634883880615, 0.9795230627059937, 3.1773641109466553, -1.5323749780654907, -3.004542350769043]

home = [0.4102410078048706, 0.5366256237030029, 1.2140278816223145, 2.9551632404327393, -1.4722083806991577, -3.4526872634887695]


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