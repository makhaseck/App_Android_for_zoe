import React, { useState, useEffect } from 'react';
import { View, Text, TouchableOpacity, Switch, StyleSheet, TextInput } from 'react-native';

import { ros } from './robot';
import ROSLIB from 'roslib';
import Slider from '@react-native-community/slider';

const App = () => {
  const [status, setStatus] = useState('connexion');
  const [titleButton, setTitleButton] = useState('deconnecter');
  const [turtlemode, setTurtlemode] = useState(true);
  const [steeringAngle, setSteeringAngle] = useState(0);
  const [accelerator, setAccelerator] = useState(0);
  const [startPosition, setStartPosition] = useState('');
  const [destination, setDestination] = useState('');
  const [timerCount, setTimerCount] = useState(0);
  const [intervalId, setIntervalId] = useState(null);
    const twist = new ROSLIB.Message({
      linear: {
        x: 0,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: 0,
      },
    });
     const cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/turtle1/cmd_vel',
      });

  const toggleSwitch = () => setTurtlemode((previousState) => !previousState);

  useEffect(() => {
    const timer = setInterval(() => {
      twist.linear.x = accelerator;
      twist.angular.z = steeringAngle;
      cmdVelTopic.publish(twist);
      setTimerCount(timerCount + 1);
      console.log('Timer Count:', timerCount, ' accelerator:', accelerator, 'steering angle:', steeringAngle);
    }, 100);

    return () => clearInterval(timer);
  }, [steeringAngle, accelerator]);
/*
  useEffect(() => {
    const timer = setInterval(() => {
      setTimerCount((prevCount) => prevCount + 1);
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    console.log('Timer Count:', timerCount);
  }, [timerCount]);
*/
  const connection = () => {
    ros.connect();
    ros.on('connection', function () {
      console.log('Connected to websocket server.');
      setStatus('Connected');
      setTitleButton('Disconnect');
    });

    ros.on('close', function () {
      console.log('Connection to rosbridge closed.');
      setStatus('Disconnected');
      setTitleButton('Connect');
    });

    ros.on('error', function (error) {
      console.log('Error connecting to rosbridge: ', error);
      setStatus('Error');
      setTitleButton('Connect');
    });
  };

  const disconnection = () => {
    if (ros) {
      ros.close();
      setStatus('Déconnecté');
      setTitleButton('Connecter');
    }
  };

  const onSteeringChange = (value) => {
    setSteeringAngle(value);
  };

  const onResetSteering = () => {
    setSteeringAngle(0);
  };

  const onAcceleratorChange = (value) => {
    setAccelerator(value);
  };

  const onGoForward = () => {
    clearInterval(intervalId);

    const interval = setInterval(() => {
      const twist = new ROSLIB.Message({
        linear: {
          x: accelerator,
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: steeringAngle,
        },
      });

      const cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/turtle1/cmd_vel',
      });
      cmdVelTopic.publish(twist);
    }, 100);

    setIntervalId(interval);
  };

  const onGoBackward = () => {
    clearInterval(intervalId);

    const interval = setInterval(() => {
      const twist = new ROSLIB.Message({
        linear: {
          x: -accelerator,
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: steeringAngle,
        },
      });

      const cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/turtle1/cmd_vel',
      });
      cmdVelTopic.publish(twist);
    }, 100);

    setIntervalId(interval);
  };

  const onPlanRoute = () => {
    console.log('Plan a route');
    console.log('Starting position:', startPosition);
    console.log('Destination:', destination);
  };

  return (
    <View style={styles.container}>
      <View style={styles.controlsContainer}>
        <Text style={styles.statusText}>Status: {status}</Text>
        <Text style={styles.modeText}>Zoe</Text>
        <Switch
          trackColor={{ false: '#767577', true: '#81b0ff' }}
          thumbColor={turtlemode ? '#f5dd4b' : '#f4f3f4'}
          ios_backgroundColor="#3e3e3e"
          onValueChange={toggleSwitch}
          value={turtlemode}
        />
        <Text style={styles.modeText}>Turtle</Text>
        <TouchableOpacity onPress={titleButton === 'Connect' ? connection : disconnection}>
          <Text>{titleButton}</Text>
        </TouchableOpacity>
      </View>

      <View style={styles.bottomContainer}>
        <View style={styles.routeContainer}>
          <Text style={styles.labelText}>Départ:</Text>
          <TextInput
            style={styles.input}
            onChangeText={(text) => setStartPosition(text)}
            value={startPosition}
          />

          <Text style={styles.labelText}>Destination:</Text>
          <TextInput
            style={styles.input}
            onChangeText={(text) => setDestination(text)}
            value={destination}
          />

          <TouchableOpacity onPress={onPlanRoute} style={styles.button}>
            <Text>Planifier un trajet</Text>
          </TouchableOpacity>
        </View>

        <View style={styles.controlsContainer}>
          <View style={styles.steeringContainer}>
            <Text style={styles.labelText}>Volant</Text>
            <View style={styles.steeringWrapper}>
              <TouchableOpacity onPress={onResetSteering} style={styles.resetButton}>
                <Text style={styles.resetButtonText}>↺</Text>
              </TouchableOpacity>
              <Slider
                style={{ width: 200 }}
                minimumValue={-90}
                maximumValue={90}
                minimumTrackTintColor="#FFFFFF"
                maximumTrackTintColor="#000000"
                onValueChange={onSteeringChange}
                value={steeringAngle}
                step={1}
              />
            </View>
            <Text>{steeringAngle}</Text>
          </View>

          <View style={styles.acceleratorContainer}>
            <Text style={styles.labelText}>Accélérateur</Text>
            <Slider
              style={{ width: 200 }}
              minimumValue={0}
              maximumValue={50}
              minimumTrackTintColor="#FFFFFF"
              maximumTrackTintColor="#000000"
              onValueChange={onAcceleratorChange}
              value={accelerator}
              step={1}
            />
            <Text>{accelerator}</Text>
          </View>
        </View>

        <View style={styles.driveContainer}>
          <TouchableOpacity onPress={onGoForward} style={styles.driveButton}>
            <Text style={styles.driveButtonText}>Avancer</Text>
          </TouchableOpacity>

          <TouchableOpacity onPress={onGoBackward} style={styles.driveButton}>
            <Text style={styles.driveButtonText}>Reculer</Text>
          </TouchableOpacity>
        </View>
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  controlsContainer: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    marginBottom: 20,
  },
  statusText: {
    fontSize: 20,
    fontWeight: 'bold',
    marginRight: 10,
  },
  modeText: {
    fontSize: 18,
    marginRight: 5,
  },
  bottomContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    marginBottom: 20,
  },
  routeContainer: {
    marginBottom: 20,
  },
  labelText: {
    fontSize: 18,
    fontWeight: 'bold',
    marginBottom: 5,
  },
  input: {
    height: 40,
    width: 200,
    borderColor: 'gray',
    borderWidth: 1,
    marginBottom: 10,
    paddingHorizontal: 10,
  },
  button: {
    backgroundColor: 'lightblue',
    paddingVertical: 10,
    paddingHorizontal: 20,
    borderRadius: 5,
  },
  steeringContainer: {
    alignItems: 'center',
    marginBottom: 20,
  },
  steeringWrapper: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  resetButton: {
    backgroundColor: '#d3d3d3',
    padding: 5,
    marginRight: 10,
    borderRadius: 3,
  },
  resetButtonText: {
    fontSize: 16,
    fontWeight: 'bold',
  },
  acceleratorContainer: {
    alignItems: 'center',
    marginBottom: 20,
  },
  driveContainer: {
    flexDirection: 'row',
    justifyContent: 'space-around',
    alignItems: 'center',
  },
  driveButton: {
    backgroundColor: 'lightblue',
    paddingVertical: 10,
    paddingHorizontal: 20,
    borderRadius: 5,
  },
  driveButtonText: {
    fontSize: 18,
    fontWeight: 'bold',
  },
});

export default App;
