Device = eth0
DebugLevel = 1
SAPSocket = $SAPSocket
gn {
	config {
		local_gn_addr = "$gn_addr"
		local_addr_conf_method = 0
		protocol_version = 0
		station_type = 0
		min_update_frequency_lpv = 1000
		max_sdu_size = 1398
		max_geo_networking_header_size = 88
		lifetime_locte = 20
	}
	location_service {
		max_retrans = 10
		retransmit_timer = 1000
		packet_buffer_size = 1024
	}
	position_service {
		host = "localhost"
		port = "2947"
	}
	beacon_service {
		retransmit_timer = 3000
		max_jitter = 150
	}
	packet_forwarding {
		default_hop_limit = 10
		max_lifetime = 600
		min_repetition_interval = 100
		geo_unicast_forwarding_algorithm = 0
		geo_broadcast_forwarding_algorithm = "$geobc_fwd_alg"
		geo_unicast_cbf_min_time = 1
		geo_unicast_cbf_max_time = 100
		default_max_communication_range = 1000
		geo_area_line_forwarding = 1
		uc_forwarding_packet_buffer_size = 256
		bc_forwarding_packet_buffer_size = 1024
		cbf_packet_buffer_size = 256
		traffic_class_scf = 0
		traffic_class_channel_offload = 0
		traffic_class_tcid = 0
	}
#	debug = 0
#	g5a_interface = "eth0"
	restrict_sender_mac_addrs = {}
}
gn6asl {
	config {
		vi_resol_addr = 1
		ts_version = "TS1.1.1"
	}
#	debug = 0
}
