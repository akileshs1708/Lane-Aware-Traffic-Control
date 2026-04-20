"""
Lane-Aware Multi-Robot Traffic Control System - Streamlit Dashboard
"""

import sys
import json
import math
import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
from collections import defaultdict

# ── page config ──────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="Lane-Aware Traffic Control",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
[data-testid="stMetricValue"] { font-size: 2rem; font-weight: 700; }
.section-header {
    font-size: 1.15rem;
    font-weight: 600;
    color: #e2e8f0;
    border-bottom: 1px solid #334155;
    padding-bottom: 6px;
    margin-bottom: 12px;
}
</style>
""", unsafe_allow_html=True)

LANE_COLOURS = {
    "normal":       "#3b82f6",
    "narrow":       "#f59e0b",
    "intersection": "#8b5cf6",
    "human_zone":   "#ef4444",
}
SAFETY_COLOURS = {"HIGH": "#22c55e", "MEDIUM": "#f59e0b", "LOW": "#ef4444"}
ROBOT_COLOURS  = px.colors.qualitative.Bold


# ── graph builders ────────────────────────────────────────────────────────────

def build_circular_graph():
    num_nodes, radius = 8, 5.0
    nodes = {}
    for i in range(num_nodes):
        a = 2 * math.pi * i / num_nodes
        nodes[i] = (radius * math.cos(a), radius * math.sin(a))
    edges = []
    for i in range(num_nodes):
        edges.append({"from": i, "to": (i+1)%num_nodes,
                      "lane_type": "narrow", "safety": "MEDIUM",
                      "max_speed": 2.0, "capacity": 1,
                      "directed": True, "requires_reservation": True})
    return nodes, edges


def build_warehouse_graph():
    nodes = {}
    for i in range(10):
        for j in range(10):
            nodes[i*10+j] = (i*2.0, j*2.0)
    edges = []
    for i in range(10):
        for j in range(9):
            fn, tn = i*10+j, i*10+j+1
            if i in [2,5,7]:    lt,sf,cap,req = "intersection","MEDIUM",1,True
            elif i==4:           lt,sf,cap,req = "human_zone",  "LOW",   1,True
            elif j in [0,8]:     lt,sf,cap,req = "narrow",      "MEDIUM",1,False
            else:                lt,sf,cap,req = "normal",       "HIGH",  2,False
            spd = 2.0 if lt=="human_zone" else 4.0
            edges.append({"from":fn,"to":tn,"lane_type":lt,"safety":sf,
                          "max_speed":spd,"capacity":cap,
                          "directed":False,"requires_reservation":req})
    for i in range(9):
        for j in range(10):
            fn, tn = i*10+j, (i+1)*10+j
            if j in [3,6]:  lt,sf,cap,req = "intersection","MEDIUM",1,True
            elif j==5:       lt,sf,cap,req = "narrow",      "MEDIUM",1,False
            else:            lt,sf,cap,req = "normal",       "HIGH",  2,False
            edges.append({"from":fn,"to":tn,"lane_type":lt,"safety":sf,
                          "max_speed":4.0,"capacity":cap,
                          "directed":False,"requires_reservation":req})
    return nodes, edges


# ── simulation ────────────────────────────────────────────────────────────────

@st.cache_data
def simulate_warehouse():
    sys.path.insert(0, "./laneawaretrafficcontrol")
    from scenario_builder import create_warehouse_scenario
    from traffic_controller import TrafficController
    graph = create_warehouse_scenario()
    ctrl  = TrafficController(graph)
    for i,(s,g) in enumerate([(0,99),(9,90),(45,54),(23,76),(5,94),(88,11),(34,65),(72,27)]):
        ctrl.add_robot(i,s,g)
    snaps = []
    for _ in range(500):
        ctrl.update()
        snaps.append({"step":ctrl.time_step,
                      "completed":ctrl.metrics["completed"],
                      "waiting":sum(1 for r in ctrl.robots.values() if r.waiting),
                      "delay":ctrl.metrics["total_delay"]})
        if ctrl.metrics["completed"]==len(ctrl.robots): break
    traj  = {rid: r.trajectory for rid,r in ctrl.robots.items()}
    usage = dict(ctrl.heatmap.usage_frequency)
    return ctrl, snaps, traj, usage


@st.cache_data
def simulate_deadlock():
    sys.path.insert(0, "./laneawaretrafficcontrol")
    from scenario_builder import create_deadlock_scenario
    from traffic_controller import TrafficController
    graph = create_deadlock_scenario()
    ctrl  = TrafficController(graph)
    for i,(s,g) in enumerate([(0,4),(1,5),(2,6),(3,7),(4,0),(5,1),(6,2),(7,3)]):
        ctrl.add_robot(i,s,g)
    snaps = []
    for _ in range(500):
        ctrl.update()
        snaps.append({"step":ctrl.time_step,
                      "completed":ctrl.metrics["completed"],
                      "waiting":sum(1 for r in ctrl.robots.values() if r.waiting),
                      "delay":ctrl.metrics["total_delay"],
                      "deadlocks":ctrl.metrics["deadlocks_resolved"]})
        if ctrl.metrics["completed"]==len(ctrl.robots): break
    traj  = {rid: r.trajectory for rid,r in ctrl.robots.items()}
    usage = dict(ctrl.heatmap.usage_frequency)
    return ctrl, snaps, traj, usage


@st.cache_data
def load_metrics():
    wh = json.load(open("./metrics_warehouse_layout.json"))
    dl = json.load(open("./metrics_circular_deadlock.json"))
    return wh, dl


# ── plotly helpers ────────────────────────────────────────────────────────────

DARK = "#0f172a"
GRID = "#1e293b"

def _base_layout(**kw):
    layout = dict(
        paper_bgcolor=DARK,
        plot_bgcolor=DARK,
        legend=dict(bgcolor=GRID, font=dict(color="#e2e8f0")),
    )
    
    # Default values (only if not overridden)
    layout.setdefault("margin", dict(l=10, r=10, t=50, b=10))
    layout.setdefault("height", 460)
    
    # Override with user values
    layout.update(kw)
    
    return layout



def fig_network(nodes, edges, title, usage=None):
    fig = go.Figure()
    for e in edges:
        x0,y0 = nodes[e["from"]]; x1,y1 = nodes[e["to"]]
        col = LANE_COLOURS.get(e["lane_type"],"#94a3b8")
        w = 2
        if usage:
            cnt = usage.get((e["from"],e["to"]),0)
            w = max(1, min(8, 1+cnt//3))
        fig.add_trace(go.Scatter(
            x=[x0,x1,None], y=[y0,y1,None], mode="lines",
            line=dict(color=col,width=w), showlegend=False,
            hovertext=f'{e["lane_type"]} | safety {e["safety"]} | speed {e["max_speed"]}',
            hoverinfo="text"))
    fig.add_trace(go.Scatter(
        x=[v[0] for v in nodes.values()], y=[v[1] for v in nodes.values()],
        mode="markers+text",
        marker=dict(size=10,color=DARK,line=dict(width=2,color="#94a3b8")),
        text=[str(k) for k in nodes], textposition="top center",
        textfont=dict(size=9,color="#cbd5e1"),
        hovertext=[f"Node {k}: ({v[0]:.1f},{v[1]:.1f})" for k,v in nodes.items()],
        hoverinfo="text", showlegend=False))
    fig.update_layout(title=dict(text=title,font=dict(color="#e2e8f0")),
                      xaxis=dict(showgrid=False,zeroline=False,showticklabels=False),
                      yaxis=dict(showgrid=False,zeroline=False,showticklabels=False),
                      **_base_layout())
    return fig


def fig_trajectories(nodes, traj, title):
    fig = go.Figure()
    for rid, pts in traj.items():
        if not pts: continue
        xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
        col = ROBOT_COLOURS[rid%len(ROBOT_COLOURS)]
        fig.add_trace(go.Scatter(x=xs,y=ys,mode="lines+markers",
            name=f"Robot {rid}", line=dict(color=col,width=2),
            marker=dict(size=4,color=col)))
        fig.add_trace(go.Scatter(x=[xs[0]],y=[ys[0]],mode="markers",showlegend=False,
            marker=dict(size=12,symbol="circle",color=col,line=dict(width=2,color="white"))))
        fig.add_trace(go.Scatter(x=[xs[-1]],y=[ys[-1]],mode="markers",showlegend=False,
            marker=dict(size=12,symbol="star",color=col,line=dict(width=2,color="white"))))
    fig.update_layout(title=dict(text=title,font=dict(color="#e2e8f0")),
                      xaxis=dict(showgrid=True,gridcolor=GRID,zeroline=False,tickfont=dict(color="#94a3b8")),
                      yaxis=dict(showgrid=True,gridcolor=GRID,zeroline=False,tickfont=dict(color="#94a3b8")),
                      **_base_layout())
    return fig

def hex_to_rgba(hex_color, alpha=0.2):
    hex_color = hex_color.lstrip('#')
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    return f"rgba({r},{g},{b},{alpha})"

def fig_timeline(snaps, key, label, colour):
    df = pd.DataFrame(snaps)
    fig = go.Figure(go.Scatter(
        x=df["step"], y=df[key], mode="lines", fill="tozeroy",
        line=dict(color=colour,width=2), name=label,
        fillcolor=hex_to_rgba(colour, 0.2)))
    fig.update_layout(
        xaxis=dict(title="Step",showgrid=True,gridcolor=GRID,tickfont=dict(color="#94a3b8")),
        yaxis=dict(title=label,showgrid=True,gridcolor=GRID,tickfont=dict(color="#94a3b8")),
        **_base_layout(height=260, margin=dict(l=10,r=10,t=10,b=10)))
    return fig


def fig_heatmap(nodes, edges, usage, title):
    fig = go.Figure()
    mx = max(usage.values(), default=1)
    for e in edges:
        x0,y0=nodes[e["from"]]; x1,y1=nodes[e["to"]]
        cnt = usage.get((e["from"],e["to"]),0)
        inten = cnt/max(mx,1)
        col = f"rgba({int(255*inten)},{int(100*(1-inten))},50,0.85)"
        fig.add_trace(go.Scatter(
            x=[x0,x1,None],y=[y0,y1,None],mode="lines",
            line=dict(color=col,width=max(1,int(inten*8))),
            hovertext=f"Lane {e['from']}->{e['to']} | usage {cnt}",
            hoverinfo="text", showlegend=False))
    fig.update_layout(title=dict(text=title,font=dict(color="#e2e8f0")),
                      xaxis=dict(showgrid=False,zeroline=False,showticklabels=False),
                      yaxis=dict(showgrid=False,zeroline=False,showticklabels=False),
                      **_base_layout())
    return fig


def fig_donut(edges, title):
    counts = defaultdict(int)
    for e in edges: counts[e["lane_type"]]+=1
    labels=list(counts.keys()); values=[counts[l] for l in labels]
    fig = go.Figure(go.Pie(
        labels=labels, values=values,
        marker_colors=[LANE_COLOURS.get(l,"#94a3b8") for l in labels],
        hole=0.55, textfont=dict(color="#e2e8f0")))
    fig.update_layout(title=dict(text=title,font=dict(color="#e2e8f0")),
                      paper_bgcolor=DARK,
                      legend=dict(bgcolor=GRID,font=dict(color="#e2e8f0")),
                      margin=dict(l=10,r=10,t=50,b=10), height=300)
    return fig


def fig_safety_bar(edges, title):
    counts = defaultdict(int)
    for e in edges: counts[e["safety"]]+=1
    order=["HIGH","MEDIUM","LOW"]
    fig = go.Figure(go.Bar(
        x=order, y=[counts.get(o,0) for o in order],
        marker_color=[SAFETY_COLOURS[o] for o in order],
        text=[counts.get(o,0) for o in order], textposition="auto",
        textfont=dict(color="#e2e8f0")))
    fig.update_layout(title=dict(text=title,font=dict(color="#e2e8f0")),
                      paper_bgcolor=DARK, plot_bgcolor=DARK,
                      xaxis=dict(tickfont=dict(color="#94a3b8")),
                      yaxis=dict(tickfont=dict(color="#94a3b8"),showgrid=True,gridcolor=GRID),
                      margin=dict(l=10,r=10,t=50,b=10), height=300)
    return fig


# ══════════════════════════════════════════════════════════════════════════════
#  SIDEBAR
# ══════════════════════════════════════════════════════════════════════════════

with st.sidebar:
    st.title("Lane-Aware Traffic Control")
    st.caption("Multi-Robot Simulation Dashboard")
    st.markdown("---")
    page = st.radio("View", [
        "Overview", "Warehouse Scenario", "Deadlock Scenario", "Code Structure"
    ], label_visibility="collapsed")
    st.markdown("---")
    st.markdown("**System Configuration**")
    st.markdown("- Robots per scenario: **8**")
    st.markdown("- Pathfinding: A* with congestion")
    st.markdown("- Deadlock detection: **DFS cycle**")


# ══════════════════════════════════════════════════════════════════════════════
#  OVERVIEW
# ══════════════════════════════════════════════════════════════════════════════

if page == "Overview":
    st.title("Lane-Aware Multi-Robot Traffic Control")
    st.caption("Simulation results and system metrics across both scenarios")
    st.markdown("---")

    wh, dl = load_metrics()

    col1, col2 = st.columns(2)
    for col, m, label in [(col1, wh, "Warehouse Layout"), (col2, dl, "Circular Deadlock")]:
        with col:
            st.markdown(f'<div class="section-header">{label}</div>', unsafe_allow_html=True)
            c1,c2,c3 = st.columns(3)
            c1.metric("Steps", m["total_steps"])
            c2.metric("Completed", f'{m["completed"]}/{m["total_robots"]}')
            c3.metric("Hotspots", m["hotspots"])
            c4,c5,c6 = st.columns(3)
            c4.metric("Total Delay", f'{m["total_delay"]:.2f}s')
            c5.metric("Avg Delay",   f'{m["average_delay"]:.2f}s')
            c6.metric("Deadlocks",   m["deadlocks_resolved"])

    st.markdown("---")
    st.markdown('<div class="section-header">Scenario Comparison</div>', unsafe_allow_html=True)
    keys   = ["total_steps","total_delay","deadlocks_resolved","hotspots"]
    labels = ["Total Steps","Total Delay (s)","Deadlocks Resolved","Hotspots"]
    fig = go.Figure(data=[
        go.Bar(name="Warehouse",       x=labels, y=[wh[k] for k in keys], marker_color="#3b82f6"),
        go.Bar(name="Circular Deadlock",x=labels, y=[dl[k] for k in keys], marker_color="#f59e0b"),
    ])
    fig.update_layout(barmode="group", paper_bgcolor=DARK, plot_bgcolor=DARK,
                      xaxis=dict(tickfont=dict(color="#94a3b8")),
                      yaxis=dict(tickfont=dict(color="#94a3b8"),showgrid=True,gridcolor=GRID),
                      legend=dict(bgcolor=GRID,font=dict(color="#e2e8f0")),
                      margin=dict(l=10,r=10,t=10,b=10), height=340)
    st.plotly_chart(fig, width='stretch')

    if dl.get("deadlock_events"):
        st.markdown("---")
        st.markdown('<div class="section-header">Deadlock Events</div>', unsafe_allow_html=True)
        for i, evt in enumerate(dl["deadlock_events"]):
            with st.expander(f"Event {i+1}  -  Timestep {evt['timestep']}"):
                st.write(f"**Cycle:** {evt['cycle']}")
                rs = {f"Robot {k}": f"Node {v}" for k,v in evt["robot_states"].items()}
                st.table(pd.DataFrame(rs, index=["At Node"]).T)

    st.markdown("---")
    st.markdown('<div class="section-header">System Architecture</div>', unsafe_allow_html=True)
    modules = [
        ("LaneGraph",          "Graph of nodes and typed directed/undirected lanes."),
        ("PathPlanner",        "A* with congestion-weighted costs and lane-type multipliers."),
        ("TrafficController",  "Per-step robot update loop with reservation management."),
        ("DeadlockDetector",   "DFS cycle detection; reroutes the shortest-remaining-path robot."),
        ("LaneHeatmap",        "Tracks per-lane usage frequency, occupancy, congestion history."),
        ("ScenarioBuilder",    "Constructs both test graphs and exports JSON metrics."),
    ]
    cols = st.columns(3)
    for idx,(name,desc) in enumerate(modules):
        with cols[idx%3]:
            st.info(f"**{name}**\n\n{desc}")


# ══════════════════════════════════════════════════════════════════════════════
#  WAREHOUSE
# ══════════════════════════════════════════════════════════════════════════════

elif page == "Warehouse Scenario":
    st.title("Warehouse Layout Scenario")
    st.caption("10x10 grid with intersections, human zones, and narrow lanes — 8 robots")

    with st.spinner("Running simulation..."):
        ctrl, snaps, traj, usage = simulate_warehouse()
    nodes, edges = build_warehouse_graph()

    tab1, tab2, tab3, tab4 = st.tabs(["Network","Robot Trajectories","Lane Heatmap","Timeline"])

    with tab1:
        cl, cr = st.columns([2,1])
        with cl:
            st.plotly_chart(fig_network(nodes, edges, "Warehouse Lane Network", usage=usage), width='stretch')
        with cr:
            st.plotly_chart(fig_donut(edges, "Lane Types"), width='stretch')
            st.plotly_chart(fig_safety_bar(edges, "Safety Levels"), width='stretch')
        st.markdown("**Lane colours:** Blue = Normal | Orange = Narrow | Purple = Intersection | Red = Human Zone")
        type_counts = defaultdict(int); type_cap = defaultdict(int)
        for e in edges:
            type_counts[e["lane_type"]]+=1; type_cap[e["lane_type"]]+=e["capacity"]
        rows=[{"Lane Type":lt.replace("_"," ").title(),"Count":type_counts[lt],
               "Total Capacity":type_cap[lt],
               "Requires Reservation":"Yes" if lt in ["intersection","human_zone"] else "No"}
              for lt in ["normal","narrow","intersection","human_zone"]]
        st.dataframe(pd.DataFrame(rows), width='stretch', hide_index=True)

    with tab2:
        st.plotly_chart(fig_trajectories(nodes, traj, "Robot Trajectories  (circles = start, stars = end)"), width='stretch')
        df_len = pd.DataFrame([{"Robot":f"Robot {rid}","Steps":len(t)} for rid,t in sorted(traj.items())])
        fig_bar = go.Figure(go.Bar(
            x=df_len["Robot"], y=df_len["Steps"],
            marker_color=[ROBOT_COLOURS[i%len(ROBOT_COLOURS)] for i in range(len(df_len))],
            text=df_len["Steps"], textposition="auto"))
        fig_bar.update_layout(paper_bgcolor=DARK, plot_bgcolor=DARK,
                              xaxis=dict(tickfont=dict(color="#94a3b8")),
                              yaxis=dict(tickfont=dict(color="#94a3b8"),showgrid=True,gridcolor=GRID),
                              margin=dict(l=10,r=10,t=10,b=10), height=260)
        st.markdown('<div class="section-header">Per-Robot Path Lengths</div>', unsafe_allow_html=True)
        st.plotly_chart(fig_bar, width='stretch')

    with tab3:
        st.plotly_chart(fig_heatmap(nodes, edges, usage, "Lane Usage Heatmap  (red = heavily used)"), width='stretch')
        if usage:
            top = sorted(usage.items(), key=lambda x:-x[1])[:15]
            st.markdown('<div class="section-header">Top 15 Most-Used Lanes</div>', unsafe_allow_html=True)
            st.dataframe(pd.DataFrame([{"Lane":f"{k[0]}->{k[1]}","Usage":v} for k,v in top]),
                         width='stretch', hide_index=True)

    with tab4:
        c1,c2 = st.columns(2)
        with c1:
            st.markdown("**Robots Completed Over Time**")
            st.plotly_chart(fig_timeline(snaps,"completed","Completed","#22c55e"), width='stretch')
        with c2:
            st.markdown("**Robots Waiting Over Time**")
            st.plotly_chart(fig_timeline(snaps,"waiting","Waiting","#f59e0b"), width='stretch')
        st.markdown("**Cumulative Delay**")
        st.plotly_chart(fig_timeline(snaps,"delay","Delay (s)","#ef4444"), width='stretch')


# ══════════════════════════════════════════════════════════════════════════════
#  DEADLOCK
# ══════════════════════════════════════════════════════════════════════════════

elif page == "Deadlock Scenario":
    st.title("Circular Deadlock Scenario")
    st.caption("8 robots on a directed circle — guaranteed deadlock, automatic resolution")

    with st.spinner("Running simulation..."):
        ctrl, snaps, traj, usage = simulate_deadlock()
    nodes, edges = build_circular_graph()
    _, dl = load_metrics()

    tab1, tab2, tab3, tab4 = st.tabs(["Network","Robot Trajectories","Timeline","Deadlock Analysis"])

    with tab1:
        cl, cr = st.columns([2,1])
        with cl:
            st.plotly_chart(fig_network(nodes, edges, "Circular Lane Network (directed)", usage=usage), width='stretch')
        with cr:
            configs = [(0,4),(1,5),(2,6),(3,7),(4,0),(5,1),(6,2),(7,3)]
            st.markdown('<div class="section-header">Robot Assignments</div>', unsafe_allow_html=True)
            st.dataframe(pd.DataFrame([{"Robot":i,"Start":s,"Goal":g} for i,(s,g) in enumerate(configs)]),
                         width='stretch', hide_index=True)
            st.markdown('<div class="section-header">Key Metrics</div>', unsafe_allow_html=True)
            st.metric("Deadlocks Resolved", dl["deadlocks_resolved"])
            st.metric("Total Steps",        dl["total_steps"])
            st.metric("Total Delay",        f'{dl["total_delay"]:.2f}s')

    with tab2:
        st.plotly_chart(fig_trajectories(nodes, traj, "Robot Trajectories  (circles = start, stars = end)"), width='stretch')

    with tab3:
        c1,c2 = st.columns(2)
        with c1:
            st.markdown("**Robots Completed**")
            st.plotly_chart(fig_timeline(snaps,"completed","Completed","#22c55e"), width='stretch')
        with c2:
            st.markdown("**Robots Waiting**")
            st.plotly_chart(fig_timeline(snaps,"waiting","Waiting","#f59e0b"), width='stretch')
        st.markdown("**Deadlocks Resolved**")
        st.plotly_chart(fig_timeline(snaps,"deadlocks","Deadlocks","#8b5cf6"), width='stretch')

    with tab4:
        st.markdown('<div class="section-header">Deadlock Event Details</div>', unsafe_allow_html=True)
        for i, evt in enumerate(dl.get("deadlock_events",[])):
            st.error(f"Deadlock at timestep **{evt['timestep']}**")
            st.write(f"**Cycle:** {' -> '.join(str(r) for r in evt['cycle'])} -> {evt['cycle'][0]}")
            rs = {f"Robot {k}": f"Stuck at Node {v}" for k,v in evt["robot_states"].items()}
            st.table(pd.DataFrame(rs, index=["Status"]).T)
            st.success("Resolved: rerouted the robot with the shortest remaining path.")
        st.markdown("---")
        st.markdown('<div class="section-header">Resolution Steps</div>', unsafe_allow_html=True)
        for title, desc in [
            ("1. Build wait-for graph",     "Each waiting robot records which robot holds its needed lane."),
            ("2. DFS cycle detection",       "Depth-first search finds circular dependencies."),
            ("3. Select victim robot",       "Robot with shortest remaining path is chosen."),
            ("4. Release reservations",      "All lanes reserved by the victim are freed."),
            ("5. Replan with avoidance",     "A* recomputes a path with congestion-avoidance, breaking the cycle."),
        ]:
            with st.expander(title):
                st.write(desc)


# ══════════════════════════════════════════════════════════════════════════════
#  CODE STRUCTURE
# ══════════════════════════════════════════════════════════════════════════════

elif page == "Code Structure":
    st.title("Codebase Overview")
    st.caption("Module breakdown, data flow, and cost function")

    files = [
        ("enums.py",                         "LaneType, SafetyLevel",                "Lane type and safety level enumerations."),
        ("data_structures.py",               "LaneMetadata, Lane, Robot",            "Core dataclasses: speeds, capacities, robot state, trajectory."),
        ("lane_graph.py",                    "LaneGraph",                            "Adjacency-list graph; auto-creates reverse edges for undirected lanes."),
        ("heatmap.py",                       "LaneHeatmap",                          "Per-lane usage frequency, occupancy, and rolling congestion history."),
        ("path_planner.py",                  "PathPlanner",                          "A* with euclidean heuristic, congestion & safety cost multipliers."),
        ("deadlock_detector.py",             "DeadlockDetector",                     "DFS cycle detection on wait-for graph; reroutes shortest-path victim."),
        ("traffic_controller.py",            "TrafficController",                    "Main loop: congestion update, deadlock check, robot movement, metrics."),
        ("scenario_builder.py",              "create_*_scenario, run_scenario",      "Builds test graphs, runs simulation loop, exports JSON metrics."),
        ("config.py",                        "Constants",                            "All tunable parameters: grid size, cost multipliers, thresholds."),
        ("main.py",                          "Entry point",                          "Calls scenario builders and visualizations."),
        ("visualizer.py",                    "Visualizer",                           "Matplotlib plots: state, heatmap, trajectories, deadlock events."),
        ("metrics_warehouse_layout.json",    "Output",                               "Saved metrics from the warehouse simulation run."),
        ("metrics_circular_deadlock.json",   "Output",                               "Saved metrics from the deadlock simulation run."),
    ]
    st.dataframe(pd.DataFrame(files, columns=["File","Key Classes / Items","Description"]),
                 width='stretch', hide_index=True)

    st.markdown("---")
    st.markdown('<div class="section-header">Data Flow</div>', unsafe_allow_html=True)
    st.code("""
ScenarioBuilder  -->  LaneGraph (nodes + typed lanes)
                 -->  TrafficController
                          |-- LaneHeatmap   (congestion tracking)
                          |-- PathPlanner   (A* planning)
                          |-- DeadlockDetector (DFS cycle)
                          |-- Robot[]       (path, trajectory, reservations)

Per step:  update()
    1. _update_congestion()    -- heatmap scores -> lane metadata
    2. _check_deadlocks()      -- build wait-for graph, DFS, reroute victim
    3. _update_robot() x N     -- reserve lane, move, update occupancy

After simulation:
    Visualizer.visualize_*()   -- matplotlib figures
    json.dump(metrics)         -- metrics_*.json
    """, language="text")

    st.markdown("---")
    st.markdown('<div class="section-header">A* Edge Cost Function</div>', unsafe_allow_html=True)
    st.latex(r"\text{cost} = \frac{d}{\text{speed}_{\text{eff}}} \times \left(1 + \frac{\text{congestion}}{100}\right) \times m_{\text{safety}} \times m_{\text{type}}")
    st.dataframe(pd.DataFrame([
        {"Condition": "Safety = LOW",          "Multiplier": 1.5},
        {"Condition": "Lane type = NARROW",    "Multiplier": 1.3},
        {"Condition": "Lane type = INTERSECTION","Multiplier": 1.2},
        {"Condition": "Lane type = HUMAN_ZONE","Multiplier": 2.0},
    ]), width='stretch', hide_index=True)

    st.markdown("---")
    st.markdown('<div class="section-header">Congestion Score</div>', unsafe_allow_html=True)
    st.latex(r"\text{congestion} = \min(100,\ \text{occupancy}\times30 + \text{usage\_freq}\times2)")
    st.caption("Effective speed = max_speed x max(0.2, 1 - congestion/100)")
