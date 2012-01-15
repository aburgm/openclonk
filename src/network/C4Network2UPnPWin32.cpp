/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2012  Nicolas Hake
 *
 * Portions might be copyrighted by other authors who have contributed
 * to OpenClonk.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * See isc_license.txt for full license and disclaimer.
 *
 * "Clonk" is a registered trademark of Matthes Bender.
 * See clonk_trademark_license.txt for full license.
 */
/* Win32 implementation of a UPnP port mapper */

#include "C4Include.h"
#include "platform/C4windowswrapper.h"
#include "network/C4Network2IO.h"
#include "network/C4Network2UPnP.h"
#include "C4Version.h"

#include <boost/foreach.hpp>
#include <natupnp.h>
#include <upnp.h>

namespace
{
	static BSTR PROTO_UDP = ::SysAllocString(L"UDP");
	static BSTR PROTO_TCP = ::SysAllocString(L"TCP");

	template<class T> inline void SafeRelease(T* &t)
	{
		if (t) t->Release();
		t = NULL;
	}
}

struct C4Network2UPnPP
{
	bool MustReleaseCOM;

	// NAT
	IStaticPortMappingCollection *mappings;
	std::set<IStaticPortMapping*> added_mappings;

	C4Network2UPnPP()
		: MustReleaseCOM(false),
		mappings(NULL)
	{}

	void AddMapping(C4Network2IOProtocol protocol, uint16_t intport, uint16_t extport);
	void RemoveMapping(C4Network2IOProtocol protocol, uint16_t extport);
	void ClearNatMappings();
};

C4Network2UPnP::C4Network2UPnP()
	: p(new C4Network2UPnPP)
{
	// Make sure COM is available
	if (FAILED(CoInitializeEx(0, COINIT_APARTMENTTHREADED)))
	{
		// Didn't work, don't do UPnP then
		return;
	}
	p->MustReleaseCOM = true;

	// Get the NAT service
	IUPnPNAT *nat = NULL;
	if (FAILED(CoCreateInstance(CLSID_UPnPNAT, NULL, CLSCTX_INPROC_SERVER, IID_IUPnPNAT, reinterpret_cast<void**>(&nat))))
		return;

	// Fetch NAT mappings
	for (int ctr = 0; ctr < 10; ++ctr)
	{
		// Usually it doesn't work on the first try, give Windows some time to query the IGD
		if (SUCCEEDED(nat->get_StaticPortMappingCollection(&p->mappings)) && p->mappings)
		{
			LogF("UPnP: Got NAT port mapping table after %d tries", ctr+1);
			break;
		}
		Sleep(1000);
	}

	SafeRelease(nat);
}

C4Network2UPnP::~C4Network2UPnP()
{
	p->ClearNatMappings();
	if (p->MustReleaseCOM)
	{
		// Decrement COM reference count
		CoUninitialize();
	}
	delete p; p = NULL;
}

void C4Network2UPnP::AddMapping(C4Network2IOProtocol protocol, uint16_t intport, uint16_t extport)
{
	p->AddMapping(protocol, intport, extport);
}

void C4Network2UPnP::ClearMappings()
{
	p->ClearNatMappings();
}

void C4Network2UPnPP::ClearNatMappings()
{
	if (!mappings)
		return;
	BOOST_FOREACH(IStaticPortMapping *mapping, added_mappings)
	{
		BSTR proto, client;
		long intport, extport;
		mapping->get_ExternalPort(&extport);
		mapping->get_InternalPort(&intport);
		mapping->get_InternalClient(&client);
		mapping->get_Protocol(&proto);
		if (SUCCEEDED(mappings->Remove(extport, proto)))
			LogF("UPnP: Closed port %d->%s:%d (%s)", extport, StdStrBuf(client).getData(), intport, StdStrBuf(proto).getData());
		::SysFreeString(proto);
		::SysFreeString(client);
		SafeRelease(mapping);
	}
	SafeRelease(mappings);
}

void C4Network2UPnPP::AddMapping(C4Network2IOProtocol protocol, uint16_t intport, uint16_t extport)
{
	if (mappings)
	{
		// Get (one of the) local host address(es)
		char hostname[MAX_PATH];
		hostent *host;
		if (gethostname(hostname, MAX_PATH) == 0 && (host = gethostbyname(hostname)) != NULL)	
		{
			in_addr addr;
			addr.s_addr = *(ULONG*)host->h_addr_list[0];

			BSTR description = ::SysAllocString(ADDL(C4ENGINECAPTION));
			BSTR client = ::SysAllocString(GetWideChar(inet_ntoa(addr)));
			IStaticPortMapping *mapping = NULL;
			if (SUCCEEDED(mappings->Add(extport, protocol == P_TCP ? PROTO_TCP : PROTO_UDP, intport, client, VARIANT_TRUE, description, &mapping)))
			{
				LogF("UPnP: Successfully opened port %d->%s:%d (%s)", extport, StdStrBuf(client).getData(), intport, protocol == P_TCP ? "TCP" : "UDP");
				added_mappings.insert(mapping);
			}
			::SysFreeString(description);
			::SysFreeString(client);
		}
	}
}
