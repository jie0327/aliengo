#!/usr/bin/env python3
"""Create an SDK2 MID-360 network configuration without replacing files."""
import argparse
import ipaddress
import json


def ipv4(value):
    try:
        address = ipaddress.IPv4Address(value)
        if address.is_unspecified or address.is_multicast or address.is_loopback:
            raise ValueError('must be a unicast interface address')
        return str(address)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('host_ip', type=ipv4)
    parser.add_argument('lidar_ip', type=ipv4)
    args = parser.parse_args()
    if args.host_ip == args.lidar_ip:
        parser.error('host and lidar IP addresses must differ')
    ports = {'cmd_data_port': 56100, 'push_msg_port': 56200,
             'point_data_port': 56300, 'imu_data_port': 56400, 'log_data_port': 56500}
    host = {}
    for name, port in ports.items():
        host[name.replace('_port', '_ip')] = '' if name == 'log_data_port' else args.host_ip
        host[name] = port + 1
    config = {'lidar_summary_info': {'lidar_type': 8},
              'MID360': {'lidar_net_info': ports, 'host_net_info': host},
              'lidar_configs': [{'ip': args.lidar_ip, 'pcl_data_type': 1, 'pattern_mode': 0,
                                 'extrinsic_parameter': dict.fromkeys(('roll', 'pitch', 'yaw', 'x', 'y', 'z'), 0)}]}
    with open(args.output, 'x', encoding='utf-8') as output:
        json.dump(config, output, indent=2)
        output.write('\n')


if __name__ == '__main__':
    main()
