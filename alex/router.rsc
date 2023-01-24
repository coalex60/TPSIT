# nov/22/2022 13:39:01 by RouterOS 7.5
# software id = 
#
/interface vlan
add interface=ether4 name=vlan20 vlan-id=20
add interface=ether4 name=vlan30 vlan-id=30
add interface=ether4 name=vlan40 vlan-id=40
/interface wireless security-profiles
set [ find default=yes ] supplicant-identity=MikroTik
/ip pool
add name=dhcp_pool0 ranges=192.168.100.2-192.168.100.126
add name=dhcp_pool1 ranges=192.168.100.130-192.168.100.190
add name=dhcp_pool2 ranges=192.168.100.194-192.168.100.254
/ip dhcp-server
add address-pool=dhcp_pool0 interface=vlan20 name=dhcp1
add address-pool=dhcp_pool1 interface=vlan30 name=dhcp2
add address-pool=dhcp_pool2 interface=vlan40 name=dhcp3
/ip address
add address=192.168.100.1/25 interface=vlan20 network=192.168.100.0
add address=192.168.100.129/26 interface=vlan30 network=192.168.100.128
add address=192.168.100.193/26 interface=vlan40 network=192.168.100.192
/ip dhcp-client
add interface=ether1
/ip dhcp-server network
add address=192.168.100.0/25 dns-server=8.8.8.8 gateway=192.168.100.1
add address=192.168.100.128/26 dns-server=8.8.8.8 gateway=192.168.100.129
add address=192.168.100.192/26 dns-server=8.8.8.8 gateway=192.168.100.193
