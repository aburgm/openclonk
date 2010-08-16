/*
	Grapple Rope
	Author: Randrian

	The rope used for grappling devices.
	Connect(obj1, obj2) connects two objects
	BreakRope() breaks the rope
	DrawIn() draws the hook in
*/

#include Library_Rope

//static const Rope_Iterations = 10;
//static const Rope_Precision = 100;
//static const Rope_PointDistance = 10;

static const Weight = 1;

// Call this to break the rope.
public func BreakRope()
{
	if(length == -1) return;
	length = -1;
	var act1 = objects[0][0];
	var act2 = objects[1][0];
	SetAction("Idle");
	// notify action targets.
	if (act1 != nil)
		act1->~OnRopeBreak();
	if (act2 != nil)
		act2->~OnRopeBreak();
	RemoveRope();
	RemoveObject();
	return;
}

/* --------------------- Callbacks form the rope ---------------------- */

/* To be overloaded for special segment behaviour */
private func CreateSegment(int index, object previous)
{
	if(index == 0) return;
	var segment;
	segment = CreateObject(GrappleRope);
	return segment;
}

/*-- Rope connecting --*/

// Connects two objects to the rope, but the length will vary on their positions.
public func Connect(object obj1, object obj2)
{
	StartRopeConnect(obj1, obj2);
	SetMaxLength(100);
	HoockAnchored = 0;

	SetAction("Hide");
	
	AddEffect("IntHang", this, 1, 1, this);
	called = 0;
	called2 = 0;
	called3 = 0;
//	Benchmark();
	return;
}

public func GetConnectStatus() { return !length_auto; }

local HoockAnchored;

/* Callback form the hook, when it hits ground */
public func HockAnchored(bool pull)
{
	if(pull)
		ConnectPull();
	HoockAnchored = 1;
}

public func HookRemoved()
{
	var new_hook = CreateObject(GrappleHook);
	new_hook->New(objects[1][0]->Contained(), this);
	objects[1][0]->SetHook(new_hook);
	objects[0][0] = new_hook;
	DrawIn();
}

/* Callback form the rope library */
public func MaxLengthReached()
{
	var clonk = objects[1][0];
	if(clonk->Contained()) clonk = clonk->Contained();
	if(!HoockAnchored)
		DrawIn();
	else if(!clonk->GetContact(-1))
	{
//		ConnectPull();
	}
}

/* for swinging */
func DoSpeed(int value)
{
	particles[-1][1][0] -= value;
}

func Benchmark()
{
	StartScriptProfiler();
	AddEffect("IntBenchmark", this, 1, 1, this);
}

func FxIntBenchmarkTimer(target, number, time)
{
	if(time >= 35*1)
	{
		StopScriptProfiler();
		Log("Called %d", called);
		Log("Called %d via Forces", called2);
		Log("Called %d via TimeStep", called3);
		return -1;
	}
	Log("%d", time);
	return;
	for(var i = 0; i < 100000; i++)
		for(var j = 0; j < 100000; j++)
			Sqrt(i*j);
}

func FxIntHangTimer() { TimeStep(); }

func TimeCompare()
{
	for(var i = 0; i < 100000; i++)
		for(var j = 0; j < 100000; j++)
			Sqrt(i*j);
}

func FxDrawInTimer()
{
	if(length < 15)
	{
		BreakRope();
		return -1;
	}
	DoLength(-5);
}

func DrawIn()
{
	if(!GetEffect("DrawIn", this))
	{
		AddEffect("DrawIn", this, 1, 1, this);
		SetFixed(0, 1);
		ConnectPull();
		var clonk = objects[1][0];
		if(clonk->Contained()) clonk = clonk->Contained();
		RemoveEffect("IntGrappleControl", clonk);
	}
}

func ConnectPull()
{
	if(length_auto == 1 && objects[1][1])
	{
		var obj = objects[1][0];
		if(obj->Contained()) obj = obj->Contained();
		// Vector r of the last segment
		var r = Vec_Sub(particles[-1][0],particles[-3][0]);
		r = [-r[1], r[0]]; // Get the orthogonal vector
		r = Vec_Normalize(r, 100); // Make it lenght 0
		var v = [obj->GetXDir(Rope_Precision), obj->GetYDir(Rope_Precision)]; // Get the speed vector
		var projection = Vec_Dot(r, Vec_Mul(v, 100))/10000; // Projekt the speed on the orthogonal vector
		obj->SetXDir(r[0]*projection/100, Rope_Precision);
		obj->SetYDir(r[1]*projection/100, Rope_Precision);
	}
	if(length_auto && objects[1][1])
	{
		var obj = objects[1][0];
		if(obj->Contained()) obj = obj->Contained();
		particles[-1][1][0] = particles[-1][0][0]-obj->SetXDir(Rope_Precision);
		particles[-1][1][1] = particles[-1][0][1]-obj->SetYDir(Rope_Precision);
	}
	_inherited(...);
	AccumulateForces();
	Log("ConnectPull");
	TimeStep();
}

local hook_angle;

func UpdateLines()
{
	var fTimeStep = 1;
	var oldangle;
	for(var i=1; i < ParticleCount; i++)
	{
		// Update the Position of the Segment
		segments[i]->SetPosition(GetPartX(i), GetPartY(i));

		// Calculate the angle to the previous segment
		var angle = Angle(particles[i][0][0], particles[i][0][1], particles[i-1][0][0], particles[i-1][0][1]);

		// Draw the left line
		var start = particles[i-1][0];
		var end   = particles[i][0];

		if(i == 1 && ParticleCount > 2)
		{
			angle = Angle(particles[2][0][0], particles[2][0][1], particles[0][0][0], particles[0][0][1]);
			end = particles[0][0];
			end[0] += -Sin(angle, 45*Rope_Precision/10);
			end[1] += +Cos(angle, 45*Rope_Precision/10);
		}
		
		if(i == 2)
		{
			angle = Angle(particles[2][0][0], particles[2][0][1], particles[0][0][0], particles[0][0][1]);
			start = particles[0][0];
			start[0] += -Sin(angle, 45*Rope_Precision/10);
			start[1] += +Cos(angle, 45*Rope_Precision/10);
		}
		
		var diff = Vec_Sub(end,start);
		var diffangle = Vec_Angle(diff, [0,0]);
		var point = Vec_Add(start, Vec_Div(diff, 2));
		var length = Vec_Length(diff)*1000/Rope_Precision/10;

		if(i == 1)
		{
			segments[i]->SetGraphics(nil, GrappleHook);
			point[0] += -Cos(diffangle, 15*Rope_Precision/10)+Sin(diffangle, 4*Rope_Precision);
			point[1] += -Cos(diffangle, 4*Rope_Precision)-Sin(diffangle, 15*Rope_Precision/10);
			length = 1000;
		}

		SetLineTransform(segments[i], -diffangle, point[0]*10-GetPartX(i)*1000,point[1]*10-GetPartY(i)*1000, length );

		// Remember the angle
		oldangle = angle;
	}
}

func GetClonkAngle()
{
	if(ParticleCount > 3)
	return Angle(particles[-1][0][0], particles[-1][0][1], particles[-3][0][0], particles[-3][0][1]);
}

local ClonkOldSpeed;

func GetClonkOff()
{
	var clonk = objects[1][0];
	var speed = [clonk->GetXDir(Rope_Precision), clonk->GetYDir(Rope_Precision)];
	var offset = speed;
	offset[0] = offset[0]*1000/Rope_Precision;
	offset[1] = offset[1]*1000/Rope_Precision;
	if(!ClonkOldSpeed)
	{
		ClonkOldSpeed = offset;
	}
	var ret = ClonkOldSpeed;
	ClonkOldSpeed = offset;
	return ret;
}

func SetSegmentTransform(obj, int r, int xoff, int yoff, int length) {
	var fsin=Sin(r, 1000), fcos=Cos(r, 1000); //length = 1200;
	// set matrix values
	obj->SetObjDrawTransform (
		+fcos,             +fsin*length/1200,             xoff, 
		-fsin, +fcos*length/1200, yoff,
	);
}

func SetLineTransform(obj, int r, int xoff, int yoff, int length, int layer, int MirrorSegments) {
	if(!MirrorSegments) MirrorSegments = 1;
	var fsin=Sin(r, 1000), fcos=Cos(r, 1000);
	// set matrix values
	obj->SetObjDrawTransform (
		+fcos*MirrorSegments, +fsin*length/1000, xoff,
		-fsin*MirrorSegments, +fcos*length/1000, yoff,layer
	);
}

protected func Definition(def) {
	SetProperty("Name", "$Name$", def);
	SetProperty("LineColors", [RGB(66,33,00) , RGB(66,33,00)], def);
	SetProperty("ActMap", {
		Hide = {
			Prototype = Action,
			Name = "Hide",
		},
	}, def);
}