// Copyright 2018 Yasu
#include "OptiTrackClient.h"
/*
 * Based on example_optitrack, which in turn is based on SimpleExample.cpp
 * provided as example in NatNetLinux.
 */

OptiTrackClient::OptiTrackClient(std::string localAddress,
                                 std::string serverAddress) {
  // set addresses
  uint32_t _localAddress = inet_addr(localAddress.c_str());
  uint32_t _serverAddress = inet_addr(serverAddress.c_str());
  // signal(SIGINT, &OptiTrackClient::stop); // this doesnt compile lol

  // Use this socket address to send commands to the server.
  struct sockaddr_in serverCommands =
      NatNet::createAddress(_localAddress, NatNet::commandPort);

  // create sockets
  sdCommand = NatNet::createCommandSocket(_localAddress);
  sdData = NatNet::createDataSocket(_localAddress);

  // start the CommandListener in a new thread
  commandListener = new CommandListener(sdCommand);
  commandListener->start();

  // Send a ping packet to the server so that it sends us the NatNet version
  // in its response to commandListener.
  NatNetPacket ping = NatNetPacket::pingPacket();
  ping.send(sdCommand, serverCommands);

  // Version number of the NatNet protocol, as reported by the server.
  unsigned char natNetMajor;
  unsigned char natNetMinor;
  // Wait here for ping response to give us the NatNet version.
  commandListener->getNatNetVersion(natNetMajor, natNetMinor);

  // Start up a FrameListener in a new thread.
  frameListener = new FrameListener(sdData, natNetMajor, natNetMinor);
  frameListener->start();
}
std::vector<RigidBody> v;
OptiTrackClient::getData() {
  bool valid;
  // Try to get a new frame from the listener.
  MocapFrame frame{frameListener->pop(&valid).first};
  if (!valid)
    return;
  std::cout << frame << '\n';
  return frame.rigidBodies();
}
int OptiTrackClient::stop() {
  frameListener->stop();
  commandListener->stop();
  frameListener->join();
  commandListener->join();

  close(sdData);
  close(sdCommand);
  return 0;
}

int main() {}
