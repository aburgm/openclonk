/**
	Power System
	Unit tests for the power system. Invokes tests by calling the 
	global function Test*_OnStart(int plr) and iterate through all 
	tests. The test is completed once Test*_Completed() returns
	true. Then Test*_OnFinished() is called, to be able to reset 
	the scenario for the next test.
	
	With LaunchTest(int nr) a specific test can be launched when
	called during runtime. A test can be skipped by calling the
	function SkipTest().
	
	@author Maikel
*/


static script_plr;

protected func Initialize()
{
	// Create a script player for some tests.
	script_plr = nil;
	CreateScriptPlayer("PowerBuddy", RGB(0, 0, 255), nil, CSPF_NoEliminationCheck);
	return;
}

protected func InitializePlayer(int plr)
{
	// Set zoom to full map size.
	SetPlayerZoomByViewRange(plr, LandscapeWidth(), nil, PLRZOOM_Direct);
	
	// No FoW to see everything happening.
	SetFoW(false, plr);
	
	// Initialize script player.
	if (GetPlayerType(plr) == C4PT_Script)
	{
		// Store the player number.
		if (script_plr == nil)
			script_plr = plr;
		// No crew needed.
		GetCrew(plr)->RemoveObject();
		return;
	}	
	
	// Move player to the start of the scenario.
	GetCrew(plr)->SetPosition(120, 150);
	
	// Some knowledge to construct a flagpole.
	GetCrew(plr)->CreateContents(Hammer);
	SetPlrKnowledge(plr, Flagpole);
	
	// Add test control effect.
	var effect = AddEffect("IntTestControl", nil, 100, 2);
	effect.testnr = 1;
	effect.launched = false;
	effect.plr = plr;
	return;
}

protected func RemovePlayer(int plr)
{
	// Remove script player.
	if (GetPlayerType(plr) == C4PT_Script)
	{
		if (plr == script_plr)
			script_plr = nil;
		return;	
	}
	return;
}


/*-- Test Control --*/

// Aborts the current test and launches the specified test instead.
global func LaunchTest(int nr)
{
	// Get the control effect.
	var effect = GetEffect("IntTestControl", nil);
	if (!effect)
	{
		// Create a new control effect and launch the test.
		effect = AddEffect("IntTestControl", nil, 100, 2);
		effect.testnr = nr;
		effect.launched = false;
		effect.plr = GetPlayerByIndex(0, C4PT_User);
		return;
	}
	// Finish the currently running test.
	Call(Format("~Test%d_OnFinished", effect.testnr));
	// Start the requested test by just setting the test number and setting 
	// effect.launched to false, effect will handle the rest.
	effect.testnr = nr;
	effect.launched = false;
	return;
}

// Calling this function skips the current test, does not work if last test has been ran already.
global func SkipTest()
{
	// Get the control effect.
	var effect = GetEffect("IntTestControl", nil);
	if (!effect)
		return;
	// Finish the previous test.
	Call(Format("~Test%d_OnFinished", effect.testnr));
	// Start the next test by just increasing the test number and setting 
	// effect.launched to false, effect will handle the rest.
	effect.testnr++;
	effect.launched = false;
	return;
}


/*-- Test Effect --*/

global func FxIntTestControlStart(object target, proplist effect, int temporary)
{
	if (temporary)
		return FX_OK;
	// Set default interval.
	effect.Interval = 2;
	return FX_OK;
}

global func FxIntTestControlTimer(object target, proplist effect)
{
	// Launch new test if needed.
	if (!effect.launched)
	{
		// Log test start.
		Log("=====================================");
		Log("Test %d started:", effect.testnr);
		// Start the test if available, otherwise finish test sequence.
		if (!Call(Format("~Test%d_OnStart", effect.testnr), effect.plr))
		{
			Log("Test %d not available, the previous test was the last test.", effect.testnr);
			Log("=====================================");
			Log("All tests have been successfully completed!");
			return -1;
		}
		effect.launched = true;
	}		
	// Check whether the current test has been finished.
	if (Call(Format("Test%d_Completed", effect.testnr)))
	{
		effect.launched = false;
		//RemoveTest();
		// Call the test on finished function.
		Call(Format("~Test%d_OnFinished", effect.testnr));
		// Log result and increase test number.
		Log("Test %d successfully completed.", effect.testnr);
		effect.testnr++;
	}
	return FX_OK;
}


/*-- Power Tests --*/

// Simple test for one steady source and one steady consumer.
global func Test1_OnStart(int plr)
{
	// Power source: one wind generator.
	SetWindFixed(50);
	CreateObjectAbove(WindGenerator, 72, 160, plr);
	
	// Power consumer: one workshop.
	var workshop = CreateObjectAbove(ToolsWorkshop, 110, 160, plr);
	workshop->CreateContents(Wood, 2);
	workshop->CreateContents(Metal, 2);
	workshop->AddToQueue(Shovel, 2);
	
	// Log what the test is about.
	Log("A steady power source (wind generator) supplying an on-demand consumer (workshop).");
	return true;
}

global func Test1_Completed()
{
	if (ObjectCount(Find_ID(Shovel)) >= 2)
		return true;
	return false;
}

global func Test1_OnFinished()
{
	// Remove wind generator, compensator and workshop.
	RemoveAll(Find_Or(Find_ID(WindGenerator), Find_ID(Compensator), Find_ID(ToolsWorkshop)));
	return;
}

// Test for one on-demand source and a few consumers.
global func Test2_OnStart(int plr)
{
	// Power source: one steam engine.
	var engine = CreateObjectAbove(SteamEngine, 100, 160, plr);
	var coal = engine->CreateContents(Coal, 1);
	coal->SetCon(10);
	
	// Power consumer: sawmill.
	CreateObjectAbove(Sawmill, 40, 160, plr);
	CreateObjectAbove(Tree_Coconut, 40, 160)->ChopDown();
	
	// Power consumer: armory.
	var armory = CreateObjectAbove(Armory, 280, 160, plr);
	armory->CreateContents(Firestone, 5);
	armory->CreateContents(Metal, 5);
	armory->AddToQueue(IronBomb, 5);
	
	// Log what the test is about.
	Log("An on-demand power source (steam engine) supplying a few on-demand power consumers (sawmill, armory).");
	return true;	
}

global func Test2_Completed()
{
	// One wood is being burned as fuel by the steam engine.
	if (ObjectCount(Find_ID(Wood)) >= 3 && ObjectCount(Find_ID(IronBomb)) >= 5)
		return true;
	return false;
}

global func Test2_OnFinished()
{
	// Remove steam engine, sawmill (possibly with wood), armory.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(Sawmill), Find_ID(Wood), Find_ID(Armory), Find_ID(Tree_Coconut)));
	return;
}

// Test for one on-demand source and one steady consumer.
global func Test3_OnStart(int plr)
{
	// Power source: one steam engine.
	var engine = CreateObjectAbove(SteamEngine, 40, 160, plr);
	engine->CreateContents(Coal, 1);
	
	// Power consumer: three pumps.
	for (var i = 0; i < 3; i++)
	{
		var pump = CreateObjectAbove(Pump, 80 + i * 20, 160, plr);
		var source = CreateObjectAbove(Pipe, 176, 292, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
		source_pipe->SetActionTargets(source, pump);
		pump->SetSource(source_pipe);
		var drain = CreateObjectAbove(Pipe, 248, 100, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
		drain_pipe->AddVertex(208, 48);
		drain_pipe->SetActionTargets(drain, pump);
		pump->SetDrain(drain_pipe);
	}
	
	// Log what the test is about.
	Log("An on-demand power source (steam engine) supplying a few steady power consumers (pumps).");
	return true;
}

global func Test3_Completed()
{
	if (GetMaterial(248, 48) == Material("Water"))
		return true;
	return false;
}

global func Test3_OnFinished()
{
	// Restore water levels.
	DrawMaterialQuad("Water", 144, 168, 208 + 1, 168, 208 + 1, 304, 144, 304, true);
	for (var x = 216; x <= 280; x++)
		for (var y = 24; y <= 120; y++)
			if (GetMaterial(x, y) == Material("Water"))
				ClearFreeRect(x, y, 1, 1);
	// Remove steam engine, pump and the pipes.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(Pump), Find_ID(Pipe)));
	return;
}

// Test one steady source with one steady consumer and a prioritized on-demand consumer.
global func Test4_OnStart(int plr)
{
	// Power source: one wind generator.
	SetWindFixed(25);
	CreateObjectAbove(WindGenerator, 40, 160, plr);
	
	// Power consumer: one pump.
	var pump = CreateObjectAbove(Pump, 124, 160, plr);
	var source = CreateObjectAbove(Pipe, 176, 292, plr);
	var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
	source_pipe->SetActionTargets(source, pump);
	pump->SetSource(source_pipe);
	var drain = CreateObjectAbove(Pipe, 248, 100, plr);
	var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
	drain_pipe->AddVertex(208, 48);
	drain_pipe->SetActionTargets(drain, pump);
	pump->SetDrain(drain_pipe);
	
	// Power connection: flagpole.
	CreateObjectAbove(Flagpole, 304, 140, plr);	 
	
	// Power consumer: one elevator.
	var elevator = CreateObjectAbove(Elevator, 372, 104, plr);
	ScheduleCall(elevator.case, "SetMoveDirection", 4 * 36, 0, COMD_Down, true);
	
	// Log what the test is about.
	Log("A steady power source (wind generator) supplying a steady power consumer (pump), while an on-demand consumer (elevator) is turned on and should be prioritized.");
	return true;
}

global func Test4_Completed()
{
	if (FindObject(Find_ID(ElevatorCase), Find_InRect(372, 230, 40, 40)))
		return true;
	return false;
}

global func Test4_OnFinished()
{
	// Restore water levels.
	DrawMaterialQuad("Water", 144, 168, 208 + 1, 168, 208 + 1, 304, 144, 304, true);
	for (var x = 216; x <= 280; x++)
		for (var y = 24; y <= 120; y++)
			if (GetMaterial(x, y) == Material("Water"))
				ClearFreeRect(x, y, 1, 1);
	// Remove wind generator, pump, the pipes, flagpole and elevator.
	RemoveAll(Find_Or(Find_ID(WindGenerator), Find_ID(Pump), Find_ID(Pipe), Find_ID(Flagpole), Find_ID(Elevator)));
	// Remove burning wood which is created when the elevator case is removed.
	Schedule(nil, "RemoveAll(Find_ID(Wood))", 1);
	return;
}

// Test one steady source with one steady consumer and a prioritized on-demand consumer (double elevators).
global func Test5_OnStart(int plr)
{
	// Power source: one wind generator.
	SetWindFixed(50);
	CreateObjectAbove(WindGenerator, 40, 160, plr);
	
	// Power consumer: one pump.
	var pump = CreateObjectAbove(Pump, 124, 160, plr);
	var source = CreateObjectAbove(Pipe, 176, 292, plr);
	var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
	source_pipe->SetActionTargets(source, pump);
	pump->SetSource(source_pipe);
	var drain = CreateObjectAbove(Pipe, 248, 100, plr);
	var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
	drain_pipe->AddVertex(208, 48);
	drain_pipe->SetActionTargets(drain, pump);
	pump->SetDrain(drain_pipe);
	
	// Power connection: flagpole.
	CreateObjectAbove(Flagpole, 304, 140, plr);	 
	
	// Power consumer: double elevator.
	var elevator1 = CreateObjectAbove(Elevator, 372, 104, plr);
	var elevator2 = CreateObjectAbove(Elevator, 434, 104, plr);
	elevator2->SetDir(DIR_Right);
	elevator2->LetsBecomeFriends(elevator1);
	ScheduleCall(elevator1.case, "SetMoveDirection", 4 * 36, 0, COMD_Down, true);
	
	// Log what the test is about.
	Log("A steady power source (wind generator) supplying a steady power consumer (pump), while an on-demand consumer (double elevator) is turned on and should be prioritized.");
	return true;
}

global func Test5_Completed()
{
	if (FindObject(Find_ID(ElevatorCase), Find_InRect(372, 230, 40, 40)))
		return true;
	return false;
}

global func Test5_OnFinished()
{
	// Restore water levels.
	DrawMaterialQuad("Water", 144, 168, 208 + 1, 168, 208 + 1, 304, 144, 304, true);
	for (var x = 216; x <= 280; x++)
		for (var y = 24; y <= 120; y++)
			if (GetMaterial(x, y) == Material("Water"))
				ClearFreeRect(x, y, 1, 1);
	// Remove wind generator, pump, the pipes, flagpole and elevator.
	RemoveAll(Find_Or(Find_ID(WindGenerator), Find_ID(Pump), Find_ID(Pipe), Find_ID(Flagpole), Find_ID(Elevator)));
	// Remove burning wood which is created when the elevator case is removed.
	Schedule(nil, "RemoveAll(Find_ID(Wood))", 1);
	return;
}

// Basic test for power storage: one steady supplier, one storage and one consumer which needs energy from both sources.
global func Test6_OnStart(int plr)
{
	// Power source: one wind generator.
	SetWindFixed(25);
	CreateObjectAbove(WindGenerator, 40, 160, plr);
	
	// Power storage: one compensator
	CreateObjectAbove(Compensator, 70, 160, plr);
	
	// Power consumer: one workshop.
	var workshop = CreateObjectAbove(ToolsWorkshop, 110, 160, plr);
	workshop->CreateContents(Wood, 2);
	workshop->CreateContents(Metal, 2);
	workshop->AddToQueue(Shovel, 2);
	
	// Log what the test is about.
	Log("A steady power source (wind generator) connected to a power storage (compensator) which are both needed to supply a steady consumer (workshop).");
	return true;
}

global func Test6_Completed()
{
	if (ObjectCount(Find_ID(Shovel)) >= 2)
		return true;
	return false;
}

global func Test6_OnFinished()
{
	// Remove wind generator, compensator and workshop.
	RemoveAll(Find_Or(Find_ID(WindGenerator), Find_ID(Compensator), Find_ID(ToolsWorkshop)));
	return;
}

// Test one overproducing on-demand producer with power storage and one steady consumer.
global func Test7_OnStart(int plr)
{
	// Power source: one steam engine.
	var engine = CreateObjectAbove(SteamEngine, 40, 160, plr);
	var coal = engine->CreateContents(Coal, 1);
	coal->SetCon(50);
	
	// Power storage: four compensators.
	CreateObjectAbove(Compensator, 20, 224, plr);
	CreateObjectAbove(Compensator, 45, 224, plr);
	CreateObjectAbove(Compensator, 70, 224, plr);
	CreateObjectAbove(Compensator, 95, 224, plr);
	
	// Power consumer: one workshop.
	var workshop = CreateObjectAbove(ToolsWorkshop, 110, 160, plr);
	workshop->CreateContents(Wood, 18);
	workshop->CreateContents(Metal, 18);
	workshop->AddToQueue(Shovel, 18);
	
	// Log what the test is about.
	Log("An on-demand power source (steam engine) whose over-production gets stored by power storage (compensators) which should provide a steady consumer (workshop).");
	return true;
}

global func Test7_Completed()
{
	if (ObjectCount(Find_ID(Shovel)) >= 18)
		return true;
	return false;
}

global func Test7_OnFinished()
{
	// Remove steam engine, compensators and workshop.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(Compensator), Find_ID(ToolsWorkshop)));
	return;
}

// Test an on-demand producer as back-up for a steady producer with for steady consumers.
global func Test8_OnStart(int plr)
{
	// Power source: one steam engine.
	var engine = CreateObjectAbove(SteamEngine, 40, 160, plr);
	engine->CreateContents(Coal, 1);
	
	// Power source: wind generators which are turned on and off again due to wind.
	CreateObjectAbove(WindGenerator, 356, 104, plr);
	SetWindFixed(80);
	Schedule(nil, "SetWindFixed(0)", 5 * 36);
	Schedule(nil, "SetWindFixed(80)", 8 * 36);
	
	// Power connection: flagpole.
	CreateObjectAbove(Flagpole, 328, 120, plr);
	
	// Power consumer: two pumps.
	for (var i = 0; i < 2; i++)
	{
		var pump = CreateObjectAbove(Pump, 92 + i * 20, 160, plr);
		var source = CreateObjectAbove(Pipe, 176, 292, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
		source_pipe->SetActionTargets(source, pump);
		pump->SetSource(source_pipe);
		var drain = CreateObjectAbove(Pipe, 248, 100, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
		drain_pipe->AddVertex(208, 48);
		drain_pipe->SetActionTargets(drain, pump);
		pump->SetDrain(drain_pipe);
	}
	
	// Log what the test is about.
	Log("An on-demand producer (steam engine) as back-up for a steady producer (wind generator) with for steady consumers (pumps).");
	return true;
}

global func Test8_Completed()
{
	if (GetMaterial(248, 48) == Material("Water"))
		return true;
	return false;
}

global func Test8_OnFinished()
{
	// Restore water levels.
	RestoreWaterLevels();
	// Remove steam engine, wind generator, flagpole, pump and the pipes.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(WindGenerator), Find_ID(Flagpole), Find_ID(Pump), Find_ID(Pipe)));
	return;
}

// Test the reduced on-demand consumer (wind mill) with an on-demand producer to always have power.
global func Test9_OnStart(int plr)
{
	// Power source: one steam engine.
	var engine = CreateObjectAbove(SteamEngine, 40, 160, plr);
	engine->CreateContents(Coal, 1);
	
	// Change the windlevels so that the engine is needed from time to time.
	SetWindFixed(50);
	Schedule(nil, "SetWindFixed(25)", 5 * 36);
	Schedule(nil, "SetWindFixed(0)", 10 * 36);
	Schedule(nil, "SetWindFixed(50)", 15 * 36);
	Schedule(nil, "SetWindFixed(0)", 20 * 36);
	
	// Power consumer: one wind mill.
	var windmill = CreateObjectAbove(Windmill, 116, 160, plr);
	windmill->CreateContents(Seeds, 3);
	windmill->AddToQueue(Flour, 3);

	// Log what the test is about.
	Log("An on-demand producer (steam engine) always provides power to an reduced on-demand consumer (wind mill).");
	return true;
}

global func Test9_Completed()
{
	if (ObjectCount(Find_ID(Flour)) >= 3)
		return true;
	return false;
}

global func Test9_OnFinished()
{
	// Remove steam engine, wind mill and flour.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(Windmill), Find_ID(Flour)));
	return;
}

// Test a double separated network and power producing pumps.
global func Test10_OnStart(int plr)
{
	// Power source (network 1): one wind generator.
	SetWindFixed(100);
	CreateObjectAbove(WindGenerator, 40, 160, plr);
	
	// Power consumer (network 1): four pumps.
	for (var i = 0; i < 4; i++)
	{
		var pump = CreateObjectAbove(Pump, 80 + i * 12, 160, plr);
		var source = CreateObjectAbove(Pipe, 168, 292, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
		source_pipe->SetActionTargets(source, pump);
		pump->SetSource(source_pipe);
		var drain = CreateObjectAbove(Pipe, 240, 100, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
		drain_pipe->AddVertex(208, 48);
		drain_pipe->SetActionTargets(drain, pump);
		pump->SetDrain(drain_pipe);
	}
	
	// Power source (network 2): four pumps.
	for (var i = 0; i < 4; i++)
	{
		var pump = CreateObjectAbove(Pump, 228 + i * 12, 160, plr);
		var source = CreateObjectAbove(Pipe, 256, 100, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 272, 24, plr);
		source_pipe->AddVertex(288, 24);
		source_pipe->AddVertex(288, 114);
		source_pipe->AddVertex(282, 120);
		source_pipe->SetActionTargets(source, pump);
		pump->SetSource(source_pipe);
		var drain = CreateObjectAbove(Pipe, 184, 292, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 208, 160, plr);
		drain_pipe->SetActionTargets(drain, pump);
		pump->SetDrain(drain_pipe);
	}
	
	// Power connection (network 2): flagpole.
	CreateObjectAbove(Flagpole, 364, 104, plr);
	
	// Power consumer (network 2): one sawmill.
	CreateObjectAbove(Sawmill, 400, 248, plr);
	for (var i = 0; i < 2; i++)
		CreateObjectAbove(Tree_Coconut, 400, 248 - 30)->ChopDown();
	
	// Log what the test is about.
	Log("Network 1 (steady producer (wind generator) supplying steady consumers (pumps) connected to network 2 where steady producers (pumps) supply an on-demand consumer (sawmill).");
	return true;
}

global func Test10_Completed()
{
	if (ObjectCount(Find_ID(Wood)) >= 5)
		return true;
	return false;
}

global func Test10_OnFinished()
{
	// Restore water levels.
	RestoreWaterLevels();
	// Remove wind generator, sawmill, wood flagpole, pump and the pipes.
	RemoveAll(Find_Or(Find_ID(WindGenerator), Find_ID(Sawmill), Find_ID(Wood), Find_ID(Flagpole), Find_ID(Pump), Find_ID(Pipe), Find_ID(Tree_Coconut)));
	return;
}

// Test connecting two networks by different allied players and then elimination of one player.
global func Test11_OnStart(int plr)
{
	// First network is owned by the player.
	var steam_engine = CreateObjectAbove(SteamEngine, 40, 160, plr);
	steam_engine->CreateContents(Coal, 4);
	
	// Second network is owned by the script player.
	SetWindFixed(50);
	CreateObjectAbove(WindGenerator, 410, 248, script_plr);
	var shipyard = CreateObjectAbove(Shipyard, 450, 248, script_plr);
	shipyard->CreateContents(Wood, 8);
	shipyard->CreateContents(Metal, 8);
	shipyard->AddToQueue(Airship, 2);
	
	// Networks are disconnected so let the script player bridge the gap.
	ScheduleCall(nil, "CreateObjectAbove", 3 * 36, 0, Compensator, 272, 160, script_plr);
	
	// Eliminate the script player and see if the normal player takes over the network correctly.
	ScheduleCall(nil, "EliminatePlayer", 6 * 36, 0, script_plr);
	
	// Rejoin the script player for other tests.
	ScheduleCall(nil, "CreateScriptPlayer", 9 * 36, 0, "PowerBuddy", RGB(0, 0, 255), nil, CSPF_NoEliminationCheck);	

	// Log what the test is about.
	Log("Two connected networks by different allied players with the elimination of one of the two players.");
	return true;
}

global func Test11_Completed()
{
	if (ObjectCount(Find_ID(Airship)) >= 2)
		return true;
	return false;
}

global func Test11_OnFinished()
{
	// Remove wind generator, steam engine, flagpole, shipyard, airship.
	RemoveAll(Find_Or(Find_ID(WindGenerator), Find_ID(SteamEngine), Find_ID(Compensator), Find_ID(Shipyard), Find_ID(Airship)));
	return;
}

// Test for the no power need rule and functionality.
global func Test12_OnStart(int plr)
{
	// Power source: one steam engine.
	var steam_engine = CreateObjectAbove(SteamEngine, 40, 160, plr);
	steam_engine->CreateContents(Coal, 4);
	
	// Power consumers: one workshop, one inventor's lab.
	var workshop = CreateObjectAbove(ToolsWorkshop, 110, 160, plr);
	workshop->CreateContents(Wood, 4);
	workshop->CreateContents(Metal, 4);
	workshop->AddToQueue(Shovel, 4);
	var lab = CreateObjectAbove(InventorsLab, 450, 248, plr);
	lab->CreateContents(Metal, 8);
	lab->AddToQueue(TeleGlove, 4);
	lab->SetNoPowerNeed(true);
	
	// Power connection: flagpole.
	CreateObjectAbove(Flagpole, 304, 140, plr);	 
	
	// Create the no power need rule.
	ScheduleCall(nil, "CreateObject", 3 * 36, 0, Rule_NoPowerNeed);
	
	// Let the lab consumer power again.
	ScheduleCall(lab, "SetNoPowerNeed", 6 * 36, 0, false);
	
	// Remove the no power need rule.
	Schedule(nil, "RemoveAll(Find_ID(Rule_NoPowerNeed))", 9 * 36, 0);

	// Log what the test is about.
	Log("No power need rule and no power need script functionality tested for a simple network.");
	return true;
}

global func Test12_Completed()
{
	if (ObjectCount(Find_ID(Shovel)) >= 4 && ObjectCount(Find_ID(TeleGlove)) >= 4)
		return true;
	return false;
}

global func Test12_OnFinished()
{
	// Remove wind generator, steam engine, flagpole, shipyard, airship.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(ToolsWorkshop), Find_ID(InventorsLab), Find_ID(Flagpole)));
	return;
}

// A network which is split up in the middle by removing a flagpol.
// TODO: this test should actually reproduce the network error.
global func Test13_OnStart(int plr)
{	
	// Power source: one steam engine.
	var steam_engine = CreateObjectAbove(SteamEngine, 36, 160, plr);
	steam_engine->CreateContents(Coal, 4);

	// Power consumer: one workshop.
	var workshop = CreateObjectAbove(ToolsWorkshop, 340, 248, plr);
	workshop->CreateContents(Wood, 2);
	workshop->CreateContents(Metal, 2);
	workshop->AddToQueue(Shovel, 2);
	
	// Power connection: one flagpole.
	CreateObjectAbove(Flagpole, 248, 280, plr);
	Schedule(nil, "RemoveAll(Find_ID(Flagpole))", 3 * 36, 0);
	ScheduleCall(nil, "CreateObjectAbove", 6 * 36, 0, Flagpole, 248, 280, plr);
	
	// Log what the test is about.
	Log("Flagpole which connects two networks is removed and created again to test network merging.");
	return true;
}

global func Test13_Completed()
{
	if (ObjectCount(Find_ID(Shovel)) >= 2)
		return true;
	return false;
}

global func Test13_OnFinished()
{
	// Remove all the structures.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(ToolsWorkshop), Find_ID(Flagpole)));
	return;
}

// Massive test which tests a lot of power structures and the performance of the system.
global func Test14_OnStart(int plr)
{
	// Start the script profiler for this test.
	StartScriptProfiler();
	
	// Power source: one steam engine.
	var steam_engine1 = CreateObjectAbove(SteamEngine, 36, 160, plr);
	steam_engine1->CreateContents(Coal, 12);

	// Power source: one steam engine.
	var steam_engine2 = CreateObjectAbove(SteamEngine, 472, 312, plr);
	
	// Power source: three wind generators.
	SetWindFixed(75);
	CreateObjectAbove(WindGenerator, 480, 104, plr);
	CreateObjectAbove(WindGenerator, 344, 104, plr);
	CreateObjectAbove(WindGenerator, 480, 248, plr);

	// Power connection: one flagpole.
	CreateObjectAbove(Flagpole, 248, 280, plr);

	// Power consumer: four pumps.
	for (var i = 0; i < 4; i++)
	{
		var pump = CreateObjectAbove(Pump, 84 + i * 12, 160, plr);
		var source = CreateObjectAbove(Pipe, 168, 292, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
		source_pipe->SetActionTargets(source, pump);
		pump->SetSource(source_pipe);
		var drain = CreateObjectAbove(Pipe, 240, 100, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
		drain_pipe->AddVertex(208, 48);
		drain_pipe->SetActionTargets(drain, pump);
		pump->SetDrain(drain_pipe);
	}
	
	// Power source: four pumps.
	for (var i = 0; i < 4; i++)
	{
		var pump = CreateObjectAbove(Pump, 228 + i * 12, 160, plr);
		var source = CreateObjectAbove(Pipe, 256, 100, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 272, 24, plr);
		source_pipe->AddVertex(288, 24);
		source_pipe->AddVertex(288, 114);
		source_pipe->AddVertex(282, 120);
		source_pipe->SetActionTargets(source, pump);
		pump->SetSource(source_pipe);
		var drain = CreateObjectAbove(Pipe, 184, 292, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 208, 160, plr);
		drain_pipe->SetActionTargets(drain, pump);
		pump->SetDrain(drain_pipe);
	}

	// Power storage: four compensators.
	CreateObjectAbove(Compensator, 20, 224, plr);
	CreateObjectAbove(Compensator, 45, 224, plr);
	CreateObjectAbove(Compensator, 70, 224, plr);
	CreateObjectAbove(Compensator, 95, 224, plr);
	CreateObjectAbove(Compensator, 70, 312, plr);
	CreateObjectAbove(Compensator, 95, 312, plr);
	
	// Power consumer: double elevator.
	var elevator1 = CreateObjectAbove(Elevator, 372, 104, plr);
	var elevator2 = CreateObjectAbove(Elevator, 434, 104, plr);
	elevator2->SetDir(DIR_Right);
	elevator2->LetsBecomeFriends(elevator1);
	ScheduleCall(elevator1.case, "SetMoveDirection", 3 * 36, 1000, COMD_Down, true);
	ScheduleCall(elevator1.case, "SetMoveDirection", 4 * 36, 1000, COMD_Up, true);

	// Power consumer: one workshop.
	var workshop = CreateObjectAbove(ToolsWorkshop, 340, 248, plr);
	workshop->CreateContents(Wood, 12);
	workshop->CreateContents(Metal, 12);
	workshop->AddToQueue(Shovel, 12);
	
	// Power consumer: one chemical lab.
	var chemical_lab = CreateObjectAbove(ChemicalLab, 28, 312, plr);
	chemical_lab->CreateContents(Firestone, 12);
	chemical_lab->CreateContents(Coal, 12);
	chemical_lab->AddToQueue(Dynamite, 12);
	
	// Power consumer: sawmill.
	CreateObjectAbove(Sawmill, 408, 312, plr);
	CreateObjectAbove(Tree_Coconut, 408, 312)->ChopDown();
	CreateObjectAbove(Tree_Coconut, 408, 312)->ChopDown();

	// Power consumer: one armory.
	var armory = CreateObjectAbove(Armory, 340, 312, plr);
	armory->CreateContents(Metal, 24);
	armory->CreateContents(Wood, 8);
	armory->AddToQueue(GrenadeLauncher, 8);

	// Power consumer: inventor's lab.
	var lab = CreateObjectAbove(InventorsLab, 430, 248, plr);
	lab->CreateContents(Metal, 40);
	lab->AddToQueue(TeleGlove, 20);

	// Log what the test is about.
	Log("Massive amount of power structures to test the performance of the power system.");
	return true;
}

global func Test14_Completed()
{
	if (ObjectCount(Find_ID(Shovel)) >= 12 && ObjectCount(Find_ID(Dynamite)) >= 12 && ObjectCount(Find_ID(GrenadeLauncher)) >= 8 && ObjectCount(Find_ID(TeleGlove)) >= 20)
		return true;
	return false;
}

global func Test14_OnFinished()
{
	// Stop the script profiler for this test.
	StopScriptProfiler();	
	// Restore water levels.
	RestoreWaterLevels();	
	// Remove all the structures.
	RemoveAll(Find_Or(Find_ID(SteamEngine), Find_ID(WindGenerator), Find_ID(ToolsWorkshop), Find_ID(ChemicalLab), Find_ID(Armory), Find_ID(Sawmill), Find_ID(Wood)));
	RemoveAll(Find_Or(Find_ID(Pump), Find_ID(Pipe), Find_ID(Compensator), Find_ID(Elevator), Find_ID(InventorsLab), Find_ID(Flagpole), Find_ID(Tree_Coconut)));
	// Remove burning wood which is created when the elevator case is removed.
	Schedule(nil, "RemoveAll(Find_ID(Wood))", 1);
	return;
}

// Test for the supported infinite pump loop, with two pumps pumping in opposite directions.
global func Test15_OnStart(int plr)
{
	// Power source: wind generator producing the power difference between the two pumps.
	SetWindFixed(10);
	CreateObjectAbove(WindGenerator, 50, 160, plr);
		
	// Power consumer: two pumps.
	for (var i = 0; i < 2; i++)
	{
		var pump = CreateObjectAbove(Pump, 80 + i * 30, 160, plr);
		var source = CreateObjectAbove(Pipe, 176, 292, plr);
		var source_pipe = CreateObjectAbove(PipeLine, 144, 160, plr);
		source_pipe->SetActionTargets(source, pump);
		pump->Call(["SetSource", "SetDrain"][i], source_pipe);
		var drain = CreateObjectAbove(Pipe, 248, 100, plr);
		var drain_pipe = CreateObjectAbove(PipeLine, 224, 48, plr);
		drain_pipe->AddVertex(208, 48);
		drain_pipe->SetActionTargets(drain, pump);
		pump->Call(["SetDrain", "SetSource"][i], drain_pipe);
	}
	
	// Some initial potential energy from water.
	CastPXS("Water", 200, 40, 248, 80);
	
	// Log what the test is about.
	Log("A supported (wind generator) infinite pump loop, with two pumps pumping in opposite directions.");
	return true;
}

global func Test15_Completed()
{
	if (GetMaterial(248, 48) == Material("Water"))
		return true;
	return false;
}

global func Test15_OnFinished()
{
	// Restore water levels.
	RestoreWaterLevels();
	// Remove steam engine, pump and the pipes.
	RemoveAll(Find_Or(Find_ID(Compensator), Find_ID(WindGenerator), Find_ID(Pump), Find_ID(Pipe)));
	return;
}


/*-- Helper Functions --*/

global func SetWindFixed(int strength)
{
	strength = BoundBy(strength, -100, 100);
	var effect = GetEffect("IntFixedWind");
	if (!effect)
		effect = AddEffect("IntFixedWind", nil, 100, 1);
	effect.strength = strength;
	return;
}

global func RestoreWaterLevels()
{
	// Restore water levels.
	DrawMaterialQuad("Water", 144, 168, 208 + 1, 168, 208 + 1, 304, 144, 304, true);
	for (var x = 216; x <= 280; x++)
		for (var y = 24; y <= 120; y++)
			if (GetMaterial(x, y) == Material("Water"))
				ClearFreeRect(x, y, 1, 1);
	return;
}

global func FxIntFixedWindTimer(object target, proplist effect)
{
	SetWind(effect.strength);
	return FX_OK;
}

global func FxIntAlternatingWindTimer(object target, proplist effect, int time)
{
	if (((time / effect.Interval) % 2) == 0)
		SetWindFixed(effect.strength);
	else
		SetWindFixed(0);
	return FX_OK;
}
	
