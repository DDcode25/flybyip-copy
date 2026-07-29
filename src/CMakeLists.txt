idf_component_register(SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_eth esp_event esp_http_server esp_netif nvs_flash driver lwip)
