var myq0 = 0, myq1 = 0;
var l1 = 60;
var l2 = 60;

var axes = {
	axis: new THREE.Vector3(),
	limits: new THREE.Vector2()
}
var link1 = {
	axis: new THREE.Vector3(),
	limits: new THREE.Vector2()
}
var link2 = {
	axis: new THREE.Vector3(),
	limits: new THREE.Vector2()
}

function setarm()
{
	link1.axis.set (0,1,0);
	link1.limits.set (-1e10, 1e10);
	link2.axis.set (0,1,0);
	link2.limits.set (-0.1, 1.3);
}

function fk (q0, q1, joint1, joint2)
{
	joint1.set (0,0,0);
	joint2.set (0,0,0);
	
	joint1.add (new THREE.Vector3(l1,0,0));
	joint1.applyAxisAngle (new THREE.Vector3(0,1,0), q0);
	
	joint2.add (new THREE.Vector3(l2,0,0));
	joint2.applyAxisAngle (new THREE.Vector3(0,1,0), q1);
	
	joint2.add (new THREE.Vector3(l1,0,0));
	joint2.applyAxisAngle (new THREE.Vector3(0,1,0), q0);
}

function myik (mytarget)
{
	var q0;
	var q1;
	
	q0 = myq0;
	q1 = myq1;
	
	var reachable = ik_ccd (mytarget, q0, q1);
	
	return reachable;
}

function drawarm()
{
	arm2.rotation.y = myq1;
	arm.rotation.y = myq0;
}

function ik_ccd (target, q0, q1)
{
	var joint1 = new THREE.Vector3();
	var joint2 = new THREE.Vector3();
	
	var end_joint = new THREE.Vector3();
	fk (q0, q1, joint1, joint2);
	end_joint.copy (joint2);
	
	var iter, i;
	for (iter = 0; iter < 1; iter ++)
	{
		for (i = 1; i >= 0; i--)
		{		
			var angle = 0, sign = 0;
			var end_j_n = new THREE.Vector3(0,0,0);
			var target_n = new THREE.Vector3(0,0,0);
			var end_j_c = new THREE.Vector3(0,0,0);
			
			var t_target = new THREE.Vector3(0,0,0);
			var t_reach = new THREE.Vector3(0,0,0);
			if (i == 1)
			{
				t_target.set (target.x-joint1.x,0,target.z-joint1.z);
				t_reach.set (end_joint.x-joint1.x,0,end_joint.z-joint1.z);
			}
			else
			{
				t_target.set (target.x,0,target.z);
				t_reach.set (end_joint.x,0,end_joint.z);
			}
			
			end_j_n.copy (t_reach);
			end_j_n.normalize();
			target_n.copy (t_target);
			target_n.normalize();
			
			angle = end_j_n.dot(target_n);
			if (angle > 1) angle = 1;
			if (angle < -1) angle = -1;
			angle = Math.acos (angle);
			
			end_j_c.copy(t_reach);
			end_j_c.cross(t_target);
			sign = end_j_c.dot(new THREE.Vector3(0,1,0));
			if (sign > 0) sign = 1;
			else sign = -1;
			
			if (i == 0) q0 += sign * angle;
			else q1 += sign * angle;
			
			// joint limit
			//q0 = q0 > Math.PI ? Math.PI : q0 < 0 ? 0 : q0;
			
			fk (q0, q1, joint1, joint2);
			end_joint.copy (joint2);
			
			var dx = new THREE.Vector3();
			var eps = 0.1;
			var target_s = new THREE.Vector3();
			target_s.copy (target);
			target_s.sub(end_joint);
			
			dx.copy (target_s);
			if (dx.length() < eps)
				return 1;
		}
	}
	paddle.position.copy (end_joint);
	paddle.position.setY (5);
	joint.position.copy (joint1);
	joint.position.setY (6);
	
	myq0 = q0;
	myq1 = q1;
	
	if (iter < 10)
		return 1;
	else
		return 0;
}