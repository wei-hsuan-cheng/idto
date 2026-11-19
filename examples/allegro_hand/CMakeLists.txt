include_directories(${PROJECT_SOURCE_DIR})

add_executable(allegro_hand allegro_hand.cc)
target_link_libraries(allegro_hand
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(allegro_hand.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/allegro_hand/allegro_hand.yaml COPYONLY)
configure_file(allegro_hand_upside_down.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/allegro_hand/allegro_hand_upside_down.yaml COPYONLY)

add_test(NAME allegro_hand_test COMMAND allegro_hand --test)

# Moving-contact gradient check (sphere-sphere contacts on the planning model, IDTO's contact model)
add_executable(contact_gradient_check contact_gradient_check.cc)
target_link_libraries(contact_gradient_check
                      drake::drake
                      trajectory_optimizer
                      find_resource
                      yaml_config
                      gflags)
