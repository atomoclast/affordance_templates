
class AffordanceTemplateStructure() :

	def __init__(self) :
		self.name = ""
		self.image = ""
		self.trajectory_map = {}
		

class AffordanceTemplateCollection() :

	def __init__(self) :
		self.class_map = {}
		self.image_map = {}
		self.file_map = {}
		self.traj_map = {}
		self.waypoint_map = {}

		
class RecognitionObjectStructure() :

	def __init__(self) :
		self.name = ""
		self.image = ""
		self.launch_file = ""
		self.package = ""
		self.marker_topic = ""

class RecognitionObjectCollection() :

	def __init__(self) :
		self.object_map = {}
		self.image_map = {}
		self.package_map = {}
		self.launch_map = {}
		self.marker_topic_map = {}
		