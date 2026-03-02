#!/usr/bin/env fish

zellij list-sessions | grep multi_pane >/dev/null
if test $status -ne 0
    echo "ERROR: no multi_pane session found"
end

zellij -s multi_pane action write 3
zellij -s multi_pane action focus-next-pane
zellij -s multi_pane action write 3

sleep 0.5
zellij -s multi_pane action write-chars "./deploy.fish slave
"
zellij -s multi_pane action focus-previous-pane

zellij -s multi_pane action write-chars "./deploy.fish master
"

