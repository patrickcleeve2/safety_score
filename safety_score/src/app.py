#!/usr/bin/env python3


import numpy as np
import pandas as pd
import streamlit as st
import plotly.express as px

from utils import *

st.set_page_config(layout="wide")
st.title("CARLA Safety Score")

st.header("Raw Data")

fname = st.text_input("Select data", value="data/data.csv")
df = pd.read_csv(fname)

df["hard_braking"] = df["field.linear_acceleration_x"] < -LONGITUDINAL_ACCEL_THRESHOLD
df["hard_braking_baseline"] = (
    df["field.linear_acceleration_x"] < -LONGITUDINAL_ACCEL_BASELINE
)
df["aggressive_turning"] = df["field.linear_acceleration_y"] > LATERAL_ACCEL_THRESHOLD
df["aggressive_turning_baseline"] = (
    df["field.linear_acceleration_y"] > LATERAL_ACCEL_BASELINE
)
df["unsafe_following"] = df["field.time_to_collision"] < UNSAFE_FOLLOWING_THRESHOLD
df["unsafe_following_baseline"] = (
    df["field.time_to_collision"] < UNSAFE_FOLLOWING_BASELINE
)

df["field.forward_collision_warning"] = df["field.forward_collision_warning"].astype(
    bool
)

st.write(df)

####################################################################################################
st.markdown("---")
df_hard_brake = df.groupby(by=["hard_braking_baseline", "hard_braking"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)

df_hard_brake_baseline = df_hard_brake[df_hard_brake.index.get_level_values(0) == True]
df_hard_brake_baseline["percentage"] = (
    df_hard_brake_baseline["count"] / df_hard_brake_baseline["count"].sum()
)


hard_braking_value = df_hard_brake_baseline[
    df_hard_brake_baseline.index.get_level_values(1) == True
]["percentage"].values[0]

cols = st.columns(2)
cols[0].subheader("hard_braking")
cols[0].write(df_hard_brake)
cols[0].write(df_hard_brake_baseline)
cols[0].write(f"hard_braking_value: {hard_braking_value}")

fig_hard_brake = px.bar(
    df_hard_brake_baseline["count"].unstack(), barmode="group", title="Hard Braking"
)
cols[1].plotly_chart(fig_hard_brake)


####################################################################################################
st.markdown("---")

df_agg_turn = df.groupby(by=["aggressive_turning_baseline", "aggressive_turning"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)

df_agg_turn_baseline = df_agg_turn[df_agg_turn.index.get_level_values(0) == True]
df_agg_turn_baseline["percentage"] = df_agg_turn_baseline["count"] / df_agg_turn_baseline["count"].sum()

aggressive_turning_value = df_agg_turn_baseline[
    df_agg_turn_baseline.index.get_level_values(1) == True
]["percentage"].values[0]

cols = st.columns(2)
cols[0].subheader("aggressive_turning")
cols[0].write(df_agg_turn)
cols[0].write(df_agg_turn_baseline)
cols[0].write(f"aggressive_turning_value: {aggressive_turning_value}")


fig_agg_turn = px.bar(
    df_agg_turn_baseline["count"].unstack(), barmode="group", title="Aggressive Turning"
)
cols[1].plotly_chart(fig_agg_turn)

####################################################################################################
st.markdown("---")

# TODO: limit to above 50mph (22 m/s)
df_unsafe_follow = df.groupby(by=["unsafe_following_baseline", "unsafe_following"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)

df_unsafe_follow_baseline = df_unsafe_follow[
    df_unsafe_follow.index.get_level_values(0) == True
]
df_unsafe_follow_baseline["percentage"] = (
    df_unsafe_follow_baseline["count"] / df_unsafe_follow_baseline["count"].sum()
)


unsafe_following_value = df_unsafe_follow_baseline[
    df_unsafe_follow_baseline.index.get_level_values(1) == True
]["percentage"].values[0]

cols = st.columns(2)
cols[0].subheader("unsafe_following")
cols[0].write(df_unsafe_follow)
cols[0].write(df_unsafe_follow_baseline)
cols[0].write(f"unsafe_following_value:  {unsafe_following_value}")

fig_unsafe_follow = px.bar(
    df_unsafe_follow_baseline["count"].unstack(),
    barmode="group",
    title="Unsafe Following",
)
cols[1].plotly_chart(fig_unsafe_follow)

####################################################################################################
st.markdown("---")
df_fcw = df.groupby(by=["field.forward_collision_warning"]).agg(
    count=pd.NamedAgg(column="field.header.stamp", aggfunc="count")
)
# fcw_value = df_fcw[df_fcw.index.get_level_values(0)==True].values[0]
fcw_value = 0

cols = st.columns(2)
cols[0].subheader("fcw")
cols[0].write(df_fcw)
cols[0].write(f"fcw_value: {fcw_value}")

fig_fcw = px.bar(df_fcw["count"], barmode="group", title="Forward Collision Warning")
cols[1].plotly_chart(fig_fcw)

####################################################################################################
st.write("---")
fd_value = 0
cols = st.columns(2)

cols[0].subheader("forced_disengagement")
cols[0].write(f"fd_value: {fd_value}")

fig_fd = px.bar([fd_value])
cols[1].plotly_chart(fig_fd)

####################################################################################################
st.markdown("----")
st.header("Predicted Collision Frequency")

score_dict = {
    "fcw": fcw_value,
    "hard_braking": hard_braking_value,
    "aggressive_turning": aggressive_turning_value,
    "unsafe_following": unsafe_following_value,
    "forced_disengagement": fd_value,
}

predicted_collision_frequency = calc_pcf(
    fcw_value=fcw_value,
    hb_value=hard_braking_value,
    at_value=aggressive_turning_value,
    uf_value=unsafe_following_value,
    fd_value=fd_value,
)
safety_score = calc_safety_score(predicted_collision_frequency)


df_score = pd.DataFrame.from_records([score_dict])
fig_pcf = px.bar(df_score, barmode="group", title="Safety Score Contributions")

cols = st.columns(2)
cols[0].markdown(f"""
The following coefficients are used in calculating the predicted collision frequency:
- FCW_COEFFICIENT: {FCW_COEFFICIENT}
- HARD_BRAKING_COEFFICIENT: {HARD_BRAKING_COEFFICIENT}
- AGGRESSIVE_TURNING_COEFFICIENT: {AGGRESSIVE_TURNING_COEFFICIENT}
- UNSAFE_FOLLOWING_COEFFICIENT: {UNSAFE_FOLLOWING_COEFFICIENT}
- FORCED_DISENGAGEMENT_COEFFICIENT: {FORCED_DISENGAGEMENT_COEFFICIENT}
""")
cols[0].write(f"Predicted Collision Frequency: {predicted_collision_frequency:.4f} collisions per 1 million miles.")
cols[0].subheader(f"Safety Score: {safety_score:.2f}")
cols[1].plotly_chart(fig_pcf)


# TODO: helper func for calculating value