from geometry_msgs.msg import PoseStamped, Pose

memoeryChip_camera_detection= [-0.3131878674030304, 0.49928998947143555, 1.9762904644012451, 0.9798975586891174, 1.3386096954345703, -1.6718480587005615]

cpu_camera_detection= [-0.3131878674030304, 0.49928998947143555, 1.9762904644012451, 0.9798975586891174, 1.3386096954345703, -1.6718480587005615]

home = [0.4102410078048706, 0.5366256237030029, 1.2140278816223145, 2.9551632404327393, -1.4722083806991577, -3.4526872634887695]


prepick_offset = 0.17
preinsert_offset = 0.17

memoeryChip_pickPose = Pose()
memoeryChip_pickPose.position.x = 0.481946058028
memoeryChip_pickPose.position.y = 0.54180890263
memoeryChip_pickPose.position.z = 0.468782527392
memoeryChip_pickPose.orientation.x = 0.364570584249
memoeryChip_pickPose.orientation.y = -0.925967095584
memoeryChip_pickPose.orientation.z = 0.0897746178097
memoeryChip_pickPose.orientation.w = 0.040171445

memoeryChip_insertPose = Pose()
memoeryChip_insertPose.position.x = 0.570468414057
memoeryChip_insertPose.position.y = 0.358130451457
memoeryChip_insertPose.position.z = 0.49
memoeryChip_insertPose.orientation.x = 0.371855843616
memoeryChip_insertPose.orientation.y = -0.928127035811
memoeryChip_insertPose.orientation.z = 0.0141112147368
memoeryChip_insertPose.orientation.w = 0.0102132553229

cpu_pickPose = Pose()
cpu_pickPose.position.x = 0.389964938732
cpu_pickPose.position.y = 0.652318908239
cpu_pickPose.position.z = 0.474
cpu_pickPose.orientation.x = 0.927640427381
cpu_pickPose.orientation.y = -0.372936390899
cpu_pickPose.orientation.z = 0.01954549467
cpu_pickPose.orientation.w = 0.00443390005975

cpu_insertPose = Pose()
cpu_insertPose.position.x = 0.623848058274
cpu_insertPose.position.y = 0.281956277387
cpu_insertPose.position.z = 0.49
cpu_insertPose.orientation.x = -0.370098384544
cpu_insertPose.orientation.y = -0.928907983007
cpu_insertPose.orientation.z = -0.0124659623909
cpu_insertPose.orientation.w = 0.00132085055592