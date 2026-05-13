/****************************************************************************
 *
 *   DDS Parameter Bridge
 *   Bridges DDS parameter_set_value_request to PX4 param_set() directly.
 *   No CONFIG_PARAM_PRIMARY or muORB required.
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_set_value_request.h>
#include <uORB/topics/parameter_set_value_response.h>
#include <lib/parameters/param.h>

#include <unistd.h>
#include <poll.h>
#include <string.h>

extern "C" __EXPORT int dds_param_bridge_main(int argc, char *argv[]);

static bool _task_should_exit = false;
static px4_task_t _task_handle = -1;

static int dds_param_bridge_thread(int argc, char *argv[])
{
	// Subscribe to DDS-bridged parameter set requests
	int req_fd = orb_subscribe(ORB_ID(parameter_primary_set_value_request));
	if (req_fd < 0) {
		PX4_ERR("[dds_param_bridge] Failed to subscribe to parameter_primary_set_value_request");
		return 1;
	}

	// Advertise parameter set response
	struct parameter_set_value_response_s rsp{};
	orb_advert_t rsp_pub = orb_advertise(ORB_ID(parameter_primary_set_value_response), &rsp);
	if (rsp_pub == nullptr) {
		PX4_ERR("[dds_param_bridge] Failed to advertise parameter_primary_set_value_response");
		orb_unsubscribe(req_fd);
		return 1;
	}

	px4_pollfd_struct_t fds[] = {
		{ .fd = req_fd, .events = POLLIN }
	};

	PX4_INFO("[dds_param_bridge] Started, waiting for DDS param requests...");

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000);

		if (ret > 0 && (fds[0].revents & POLLIN)) {
			struct parameter_set_value_request_s req;
			orb_copy(ORB_ID(parameter_primary_set_value_request), req_fd, &req);

			param_t param = req.parameter_index;
			bool success = false;

			// Validate parameter index
			if (param == PARAM_INVALID) {
				PX4_ERR("[dds_param_bridge] Invalid parameter index: %u", req.parameter_index);
			} else {
				switch (param_type(param)) {
				case PARAM_TYPE_INT32:
					success = (param_set(param, &req.int_value) == PX4_OK);
					break;

				case PARAM_TYPE_FLOAT:
					success = (param_set(param, &req.float_value) == PX4_OK);
					break;

				default:
					PX4_ERR("[dds_param_bridge] Unsupported type for %s", param_name(param));
					break;
				}
			}

			// Send response
			rsp.timestamp = hrt_absolute_time();
			rsp.request_timestamp = req.timestamp;
			rsp.parameter_index = req.parameter_index;
			orb_publish(ORB_ID(parameter_primary_set_value_response), rsp_pub, &rsp);

			if (success) {
				PX4_INFO("[dds_param_bridge] Set %s OK", param_name(param));
			} else {
				PX4_WARN("[dds_param_bridge] Set %s FAILED", param_name(param));
			}
		}
	}

	orb_unsubscribe(req_fd);
	PX4_INFO("[dds_param_bridge] Stopped");
	return 0;
}

int dds_param_bridge_main(int argc, char *argv[])
{
	if (argc > 1 && strcmp(argv[1], "stop") == 0) {
		_task_should_exit = true;
		if (_task_handle != -1) {
			px4_task_delete(_task_handle);
			_task_handle = -1;
		}
		return 0;
	}

	if (argc > 1 && strcmp(argv[1], "status") == 0) {
		PX4_INFO("[dds_param_bridge] Running: %s", _task_handle != -1 ? "yes" : "no");
		return 0;
	}

	if (_task_handle != -1) {
		PX4_WARN("[dds_param_bridge] Already running");
		return 0;
	}

	_task_should_exit = false;
	_task_handle = px4_task_spawn_cmd("dds_param_bridge",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_PARAMS,
					  2048,
					  dds_param_bridge_thread,
					  nullptr);

	return 0;
}
