/*
 * The MIT License (MIT)
 * Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co£¬Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

// test_server.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <signal.h>
#include <mutex>
#include <libwebsockets.h>
#include "drvver.h"
#include "VRTrackedDeviceMgr.h"

#pragma comment(lib,"Ws2_32.lib")

using namespace std;
using namespace Hypereal;

Hypereal::VRTrackedDeviceMgr* pMgr = NULL;
list<struct lws*> on_line_clint;
std::mutex mtx;
static int destroy_flag = 0;


/* *
* websocket_write_back: write the string data to the destination wsi.
*/
int websocket_write_back(struct lws *wsi_in, char *str, int str_size_in)
{
	if (str == NULL || wsi_in == NULL)
		return -1;

	int n;
	int len;
	char *out = NULL;

	if (str_size_in < 1)
		len = strlen(str);
	else
		len = str_size_in;

	out = (char *)malloc(sizeof(char)*(LWS_SEND_BUFFER_PRE_PADDING + len + LWS_SEND_BUFFER_POST_PADDING));
	//* setup the buffer*/
	memcpy(out + LWS_SEND_BUFFER_PRE_PADDING, str, len);
	//* write out*/
	n = lws_write(wsi_in, (unsigned char*)out + LWS_SEND_BUFFER_PRE_PADDING, len, LWS_WRITE_TEXT);

	//* free the buffer*/
	free(out);

	return n;
}


void daemon()
{
	int k = 0;
	while (true)
	{

		char buff[0xfff];
		mtx.lock();
		for (struct lws* n : on_line_clint)
		{
			websocket_write_back(n, (char*)pMgr->dump(), -1);
		}
		mtx.unlock();
		Sleep(1000);
	}
}


static int ws_service_callback(
struct lws *wsi,
enum lws_callback_reasons reason, void *user,
	void *in, size_t len)
{

	switch (reason) {

	case LWS_CALLBACK_ESTABLISHED:
		mtx.lock();
		on_line_clint.push_back(wsi);
		mtx.unlock();
		break;

		//* If receive a data from client*/
	case LWS_CALLBACK_RECEIVE:
	{
		/*
		websocket_write_back(wsi, (char *)get_ver(), -1);
		websocket_write_back(wsi, (char *)pMgr->dump(), -1);
		*/
		break;
	}
	case LWS_CALLBACK_CLOSED:
		mtx.lock();
		on_line_clint.remove(wsi);
		mtx.unlock();
		break;

	default:
		break;
	}

	return 0;
}

struct per_session_data {
	int fd;
};

static struct lws_protocols protocols[] = {
	{
		"my-echo-protocol",
		ws_service_callback,
		sizeof(struct per_session_data),
		0, /* rx buf size must be >= permessage-deflate rx size */
	},
	{ NULL, NULL, 0, 0 } /* terminator */
};

int setup(void)
{
	// server url will usd port 5000
	int port = 5000;
	const char *interface = NULL;
	struct lws_context_creation_info info;

	struct lws_context *context;
	// Not using ssl
	const char *cert_path = NULL;
	const char *key_path = NULL;
	// no special options
	int opts = 0;


	//* register the signal SIGINT handler */


	//* setup websocket protocol */

	//* setup websocket context info*/
	memset(&info, 0, sizeof info);
	info.port = port;
	//info.iface = interface;
	info.protocols = protocols;
	info.extensions = lws_get_internal_extensions();
	info.ssl_cert_filepath = cert_path;
	info.ssl_private_key_filepath = key_path;
	info.gid = -1;
	info.uid = -1;
	info.max_http_header_pool = 16;
	info.options = opts | LWS_SERVER_OPTION_VALIDATE_UTF8;

	//* create libwebsocket context. */
	context = lws_create_context(&info);
	if (context == NULL) {
		return -1;
	}


	//* websocket service */
	while (!destroy_flag) {
		lws_service(context, 1000);
	}
	Sleep(10);
	lws_context_destroy(context);

	return 0;
}

int setup_async(VRTrackedDeviceMgr *_pMgr)
{
	pMgr = _pMgr;
	std::thread t1;
	t1 = std::thread(setup);
	t1.detach();

	std::thread t2;
	t2 = std::thread(daemon);
	t2.detach();
	return 0;
}