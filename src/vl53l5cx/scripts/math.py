#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

        max_sensor1 = []
        max_sensor2 = []
        max_sensor3 = []
        max_sensor4 = []
        for i in range(0, lenght):
            if self.max_noise_data_1[i] > self.sensor_1[i]:
                max_sensor1.append(self.max_noise_data_1[i])
            else:
                max_sensor1.append(self.sensor_1[i])

        for i in range(0, lenght):
            if self.max_noise_data_2[i] > self.sensor_2[i]:
                max_sensor2.append(self.max_noise_data_2[i])
            else:
                max_sensor2.append(self.sensor_2[i])

        for i in range(0, lenght):
            if self.max_noise_data_3[i] > self.sensor_3[i]:
                max_sensor3.append(self.max_noise_data_3[i])
            else:
                max_sensor3.append(self.sensor_3[i])

        for i in range(0, lenght):
            if self.max_noise_data_4[i] > self.sensor_4[i]:
                max_sensor4.append(self.max_noise_data_4[i])
            else:
                max_sensor4.append(self.sensor_4[i])

        self.save_data_range(
            max_sensor1,
            max_sensor2,
            max_sensor3,
            max_sensor4,
            self.max_noise,
        )