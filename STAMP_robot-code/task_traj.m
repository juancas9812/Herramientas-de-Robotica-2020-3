function [trajf]=task_traj(Vk,pr,Ts,t0)
trajf=[];
trajf=[trajf;traj_planner(1,[-0.3,0.65,0.15;-0.3,0.65,0.1],Vk,pr,Ts,t0)];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.65,0.1;-0.3,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.65,0.15;-0.3,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.55,0.15;-0.3,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.55,0.1;-0.3,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.55,0.15;-0.3,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.45,0.15;-0.3,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.45,0.1;-0.3,0.45,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.3,0.45,0.15;-0.2,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.45,0.15;-0.2,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.45,0.1;-0.2,0.45,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.45,0.15;-0.2,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.55,0.15;-0.2,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.55,0.1;-0.2,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.55,0.15;-0.2,0.65,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.65,0.15;-0.2,0.65,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.65,0.1;-0.2,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.2,0.65,0.15;-0.1,0.65,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.65,0.15;-0.1,0.65,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.65,0.1;-0.1,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.65,0.15;-0.1,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.55,0.15;-0.1,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.55,0.1;-0.1,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.55,0.15;-0.1,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.45,0.15;-0.1,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.45,0.1;-0.1,0.45,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[-0.1,0.45,0.15;0   ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.45,0.15;0   ,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.45,0.1;0   ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.45,0.15;0   ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.55,0.15;0   ,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.55,0.1;0   ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.55,0.15;0   ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.65,0.15;0   ,0.65,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.65,0.1;0   ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0   ,0.65,0.15;0.1 ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.65,0.15;0.1 ,0.65,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.65,0.1;0.1 ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.65,0.15;0.1 ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.55,0.15;0.1 ,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.55,0.1;0.1 ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.55,0.15;0.1 ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.45,0.15;0.1 ,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.45,0.1;0.1 ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.1 ,0.45,0.15;0.2 ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.45,0.15;0.2 ,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.45,0.1;0.2 ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.45,0.15;0.2 ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.55,0.15;0.2 ,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.55,0.1;0.2 ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.55,0.15;0.2 ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.65,0.15;0.2 ,0.65,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.65,0.1;0.2 ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.2 ,0.65,0.15;0.3 ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.65,0.15;0.3 ,0.65,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.65,0.1;0.3 ,0.65,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.65,0.15;0.3 ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.55,0.15;0.3 ,0.55,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.55,0.1;0.3 ,0.55,0.15],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.55,0.15;0.3 ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.45,0.15;0.3 ,0.45,0.1],Vk,pr,Ts,trajf(r,c))];
[r,c]=size(trajf);
trajf=[trajf;traj_planner(1,[0.3 ,0.45,0.1;0.3 ,0.45,0.15],Vk,pr,Ts,trajf(r,c))];

end