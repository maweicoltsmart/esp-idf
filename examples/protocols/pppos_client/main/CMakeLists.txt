idf_component_register(SRCS "pppos_client_main.c" "leds.c" "inputio.c" "flow_meter.c" "param.c" "crc32.c" "mydns.c" "fireware_update.c" "hex.c"
# Embed the server root certificate into the final binary
                    	EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                    INCLUDE_DIRS ".")
