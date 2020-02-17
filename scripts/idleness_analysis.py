#!/usr/bin/env python

"""
    this file holds the serializer and deserializer of the idlenesses of the Destinations.
    IdlenessLogger:
        serializer, called during the shutdown of topoplanner.py
        + pickles the Destinations in the appropriate "idleness/$(env)/dumps/$(robotnum)/" subfolder
        + prints a PrettyTable in the appropriate "idleness/$(env)/$(robotnum)/" subfolder
        
    IdlenessAnalizer:
        deserializer, called when running this file alone (NOT AS NODE)
        + unpickles the Destinations
        + analyzes the Destinations
        + shows a graphical result
"""

import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
import tkMessageBox
import time
import subprocess
import pickle
import tkFont
from pprint import pprint

from destination import Destination
from prettytable import PrettyTable
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk


DEFAULT_PATH = (os.path.dirname(os.path.realpath(__file__))).replace('scripts', 'idleness/')


class IdlenessLogger(object):
    def __init__(self, dest_list, environment, robots_num, path=DEFAULT_PATH):
        if all(isinstance(d, Destination) for d in dest_list):
            self.dest_list = dest_list
        else:
            raise ValueError("Items of dest_list aren't of type <Destination>")
        self.path = path
        self.environment = environment  # office, house ...
        self.robots_num = robots_num
        self.tk_root = tk.Tk()
    
    def show_confirm_gui(self):
        self.tk_root.title("Dump destinations")
        self.tk_root.geometry("320x200")
        self.tk_root.eval('tk::PlaceWindow %s center' % self.tk_root.winfo_toplevel())
        
        pop = subprocess.Popen(["wmctrl", "-r", "Dump destinations", "-b", "add,above"])
        pop.communicate()
        
        msg = "Do you want to save\n the observed idlenesses\n of the destinations?"
        font = tkFont.Font(family="Helvetica", size=14)
        
        label = tk.Label(self.tk_root, text=msg, font=font)
        label.pack(side="top", fill="both", expand=True, padx=20, pady=20)
        
        frame = tk.Frame(self.tk_root).pack(side="bottom", expand=True)
        
        button = tk.Button(frame, text="OK", command=lambda: self.write_statfile())
        button.pack(side="left", fill="none", expand=True, padx=5, pady=5)
        
        button = tk.Button(frame, text="Cancel", command=lambda: self.tk_root.destroy())
        button.pack(side="right", fill="none", expand=True, padx=5, pady=5)
        
        self.tk_root.mainloop()
    
    def write_statfile(self):
        datetime = time.strftime("%d-%m@%H:%M", time.localtime())
        subdir = "%s/%s/" % (self.environment, self.robots_num)
        filename = "%s-%s-%sbots.txt" % (datetime, self.environment, self.robots_num)
        
        self.write_dumpfile(filename=filename, env=self.environment, robots=str(self.robots_num))
        
        lines = []
        pt = PrettyTable()
        pt.field_names = ['Dest name', 'true', 'remaining', 'estimated', 'path_len', 'mean', 'max', 'min']
        
        total_visits = 0
        means = []
        for d in sorted(self.dest_list, key=lambda dest: dest.name):
            d.force_shutdown()
            
            observations = d.get_stats()
            true_idls = [o.idleness.get_true() for o in observations]
            
            # try:
            # if len(idlenesses) > 0:
            mean = round(np.mean(true_idls), 3)
            min = round(np.min(true_idls), 3)
            max = round(np.max(true_idls), 3)
        
            means.append(mean)
            total_visits += len(d.get_visits())
            
            for i in range(len(observations)):
                idl = observations[i].idleness
                if i == 0:
                    pt.add_row([
                        d.name, idl.get_true(),
                        idl.get_remaining(), idl.get_estimated(), observations[i].path_len, mean, max, min
                    ])
                else:
                    pt.add_row([
                        '', idl.get_true(),
                        idl.get_remaining(), idl.get_estimated(),observations[i].path_len, '', '', ''
                    ])
            # else:
            #     rospy.logerr("%s idlenesses empty" % d.name)
            # except ValueError:
            #     rospy.logwarn("Something wrong while calculating idlenesses stats")
            #     print true_idls
        
        separator = "-----------\n"
        lines.append(pt.__str__())
        lines.append("\nAverage idleness: %s\n" % round(np.mean(means), 3))
        lines.append(separator + "Variance idleness: %s\n" % round(np.var(means), 3))
        lines.append(separator + "Total visits: %s\n" % total_visits)
        
        file = open(self.path + subdir + filename, 'w')
        file.writelines(lines)
        
        rospy.loginfo('Destination idlenesses have been wrote to %s' % self.path)
        self.tk_root.destroy()
    
    def write_dumpfile(self, filename, env, robots):
        name = filename.split('.')
        name.insert(1, '_DUMP.')
        _filename = ''.join(name)
        
        dests = []
        for d in sorted(self.dest_list, key=lambda dest: dest.name):
            dests.append(d)
        
        f = open(self.path+env+"/dumps/"+robots+'/'+_filename, 'w')
        pickle.dump(dests, f)
        f.close()


class IdlenessAnalizer(object):
    def __init__(self, maindir=DEFAULT_PATH):
        self.maindir = maindir
    
    @staticmethod
    def load(filename):
        """
        loads the serialized destinations from the path+filename provided
        :param filename: (str) name of the file
        :return: list of the deserialized destinations
        """
        if not filename.endswith("_DUMP.txt"):
            name = filename.split(".")
            return pickle.load(open(name[0]+"_DUMP.txt", 'rb'))
        
        return pickle.load(open(filename, 'rb'))
    
    def interferences(self):
        environments = os.listdir(self.maindir)
        environment_averages = []
    
        for env in sorted(environments):
            robots_num = os.listdir(self.maindir + env)
            robots_num.remove('dumps')
            robots_averages = []
        
            for robot in sorted(robots_num):
                runs = os.listdir(self.maindir + env + '/dumps/' + robot)
                runs_interf_avgs = []
                true_idl_avgs = []
                observs_num = 0
            
                for run in runs:
                    dests = self.load(self.maindir + env + '/dumps/' + robot + '/' + run)
                    observs = []
                
                    for d in dests:
                        if d.get_visits():  # != []
                            observs.extend(d.get_visits())
                
                    if observs:  # != []
                        observs_num += len(observs)
                        runs_interf_avgs.append(np.mean([o.get_interference() for o in observs]))
                        true_idl_avgs.append(np.mean([o.idleness.get_true() for o in observs]))
            
                if runs_interf_avgs:  # != []
                    robots_averages.append({
                        'robot_num': int(robot),
                        'avg interf': round(np.mean(runs_interf_avgs), 4),
                        'avg idleness': round(np.mean(true_idl_avgs), 4),
                        'visits': observs_num
                    })
        
            environment_averages.append({'environment': env, 'stats': robots_averages})
    
        return environment_averages
    
    @staticmethod
    def single_plot(robot_range, env):
        fig, axs = plt.subplots(3)
    
        avg_interf = [e['avg interf'] for e in env['stats']]
        axs[0].plot(robot_range, avg_interf, 'r')
        axs[0].set_title('Average interference')
    
        avg_idl = [e['avg idleness'] for e in env['stats']]
        axs[1].plot(robot_range, avg_idl, 'g')
        axs[1].set_title('Average idleness')
    
        visits = [e['visits'] for e in env['stats']]
        axs[2].plot(robot_range, visits, 'b')
        axs[2].set_title('Visits number')
    
        fig.suptitle(env['environment'])
        
        plt.show()
    
    @staticmethod
    def three_plots(robot_range, env_averages):
        # ticks = plt.xticks(fontsize=25)
        for env in env_averages:
            # plt.xlabel("Robot number", fontsize="xx-large")
            # plt.ylabel("Interference", fontsize="xx-large")
            fig, axs = plt.subplots(3)
            
            avg_interf = [e['avg interf'] for e in env['stats']]
            avg_idl = [e['avg idleness'] for e in env['stats']]
            visits = [e['visits'] for e in env['stats']]
            
            axs[0].plot(robot_range, avg_interf, 'r', marker="o")
            # axs[0].set_xticks(ticks)
            axs[0].set_title('Average interference', fontsize=18)
            
            axs[1].plot(robot_range, avg_idl, 'g', marker="o")
            # axs[1].set_xticks(ticks)
            axs[1].set_title('Average idleness', fontsize=18)
            
            axs[2].plot(robot_range, visits, 'b', marker="o")
            # axs[2].set_xticks(ticks)
            axs[2].set_title('Visits number', fontsize=18)
            
            fig.suptitle(env['environment'], fontsize=22)
            fig.tight_layout()

        plt.show(block=False)
        try:
            input('Press enter to close')
        except:
            pass
        plt.close('all')


if __name__ == '__main__':
    robot_range = [3, 4, 5, 6, 7, 8]

    ia = IdlenessAnalizer()
    environmental_interferences = ia.interferences()
    # pprint(environmental_interferences)
    ia.three_plots(robot_range, environmental_interferences)
