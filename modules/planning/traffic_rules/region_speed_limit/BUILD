load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility=["//visibility:public"])

filegroup(
    name="region_speed_limit_files",
    srcs=glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name="libregion_speed_limit.so",
    srcs=[
        "region_speed_limit.cc",
    ],
    hdrs=[
        "region_speed_limit.h",
    ],
    description=":plugin_region_speed_limit_description.xml",
    deps=[
        "//cyber",
        "//modules/map:apollo_map",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_proto",
    ],
)

apollo_package()

cpplint()
